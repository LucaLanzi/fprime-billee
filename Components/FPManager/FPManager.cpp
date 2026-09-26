// ======================================================================
// \title  FPManager.cpp
// \brief  cpp file for FPManager component implementation class
// ======================================================================

#include "Components/FPManager/FPManager.hpp"

namespace Billee {

namespace {
//! Consecutive-sample counter step: count up (saturating) while out of bounds, reset otherwise
U8 step(U8 count, bool outOfBounds) {
    if (!outOfBounds) {
        return 0;
    }
    return (count < 255) ? static_cast<U8>(count + 1) : count;
}
//! A debounce setting of 0 would never trip; treat it as 1
U8 atLeastOne(U8 value) {
    return (value == 0) ? 1 : value;
}
}  // namespace

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

FPManager ::FPManager(const char* const compName) : FPManagerComponentBase(compName) {
    // Map each INA780 (InaSensorId - 1) to its subsystem
    for (U8 i = 0; i < 6; i++) {
        this->m_sensor[i].subIndex = SUB_DRIVETRAIN;
    }
    this->m_sensor[6].subIndex = SUB_ARM;
    this->m_sensor[7].subIndex = SUB_SCIENCE;
    this->m_sensor[8].subIndex = SUB_LOGIC;
}

FPManager ::~FPManager() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void FPManager ::powerReadingIn_handler(FwIndexType portNum,
                                        const Billee::Subsystems& subsystem,
                                        const Billee::PowerReading& reading) {
    (void)portNum;
    (void)subsystem;  // the sensor id is authoritative for the subsystem mapping
    this->loadParamsIfNeeded();
    this->applyPendingPowerStates();

    const Billee::InaSensorId sensorId = reading.get_sourceId();
    const U32 raw = static_cast<U32>(sensorId.e);
    if (raw < 1 || raw > NUM_INA) {
        return;
    }
    SensorEval& eval = this->m_sensor[raw - 1];
    const U8 sub = eval.subIndex;

    // The LOGIC reading arrives last in each InaManager burst: republish the bounds once per burst
    // so a GDS client that connects after boot still sees them.
    if (sensorId == Billee::InaSensorId::LOGIC) {
        this->publishThresholdTelemetry();
    }

    if (!reading.get_valid()) {
        // Sensor loss is alert-only (InaManager reports it). Never evaluate stale/zeroed values.
        eval.oc = 0;
        eval.ov = 0;
        eval.uv = 0;
        this->writeSensorStateTelemetry(sensorId, Billee::FaultState::STALE);
        return;
    }

    const Billee::PowerBounds& bounds = this->m_bounds[sub];
    const F32 voltage = reading.get_voltage();
    const F32 current = reading.get_current();

    const bool ocNow = (current > bounds.get_currentFaultHigh()) || (current < -bounds.get_currentFaultHigh());
    const bool ovNow = voltage > bounds.get_vbusFaultHigh();

    // Undervoltage gate: DRIVE/ARM/SCIENCE VBUS is on the switched side, so it is only meaningful
    // while commanded ON and after the settle time. LOGIC measures its battery directly.
    const bool uvArmed = (sub == SUB_LOGIC) ||
                         (this->m_commandedOn[sub] &&
                          (this->nowSeconds() - this->m_onSince[sub]) >= static_cast<F64>(this->m_uvSettleS));
    const bool uvNow = uvArmed && (voltage < bounds.get_vbusFaultLow());

    eval.oc = step(eval.oc, ocNow);
    eval.ov = step(eval.ov, ovNow);
    eval.uv = step(eval.uv, uvNow);

    this->writeSensorStateTelemetry(
        sensorId, (ocNow || ovNow || uvNow) ? Billee::FaultState::TRIPPED : Billee::FaultState::NOMINAL);

    // Fire whenever a counter is at/over its debounce threshold and the latch isn't set yet.
    // Level-triggered on purpose: if the state-machine signal was dropped on a full queue, the
    // next out-of-bounds sample sends it again. Once FAULTED, nothing more is sent.
    // Overcurrent takes priority in the reported reason.
    const U8 dOc = atLeastOne(this->m_debounceOc);
    const U8 dOv = atLeastOne(this->m_debounceOv);
    const U8 dUv = atLeastOne(this->m_debounceUv);
    bool fire = false;
    Billee::FaultReason reason = Billee::FaultReason::OVERCURRENT;
    if (eval.oc >= dOc) {
        fire = true;
        reason = Billee::FaultReason::OVERCURRENT;
    } else if (eval.ov >= dOv) {
        fire = true;
        reason = Billee::FaultReason::OVERVOLTAGE;
    } else if (eval.uv >= dUv) {
        fire = true;
        reason = Billee::FaultReason::UNDERVOLTAGE;
    }

    if (fire && !this->isFaulted(sub)) {
        this->log_WARNING_HI_PowerFaultDetail(sensorId, reason, voltage, current);
        this->sendFault(sub, reason);
    }
}

void FPManager ::thermalReadingIn_handler(FwIndexType portNum,
                                          const Billee::Subsystems& subsystem,
                                          const Billee::ThermalReading& reading) {
    (void)portNum;
    this->loadParamsIfNeeded();
    this->applyPendingPowerStates();

    U8 sub = 0;
    if (!subIndexFor(subsystem, sub)) {
        return;
    }

    const Billee::ThermalStates state = reading.get_tempState();
    const U32 mcpRaw = static_cast<U32>(reading.get_sensorId());

    if (state == Billee::ThermalStates::FAILURE) {
        // Alert only (Luca, 2026-09-26): an unreadable sensor never trips. The shared arm/science
        // sensor arrives twice per cycle; report each physical sensor once.
        if (mcpRaw >= 1 && mcpRaw <= NUM_MCP && !this->m_mcpLost[mcpRaw - 1]) {
            this->m_mcpLost[mcpRaw - 1] = true;
            this->log_WARNING_HI_ThermalSensorLost(reading.get_sensorId());
        }
        this->m_thermalCount[sub] = 0;
        this->m_thermalFault[sub] = false;
        return;
    }

    if (mcpRaw >= 1 && mcpRaw <= NUM_MCP && this->m_mcpLost[mcpRaw - 1]) {
        this->m_mcpLost[mcpRaw - 1] = false;
        this->log_ACTIVITY_HI_ThermalSensorRecovered(reading.get_sensorId());
    }

    const bool faultNow = (state == Billee::ThermalStates::FAULT);
    this->m_thermalCount[sub] = step(this->m_thermalCount[sub], faultNow);
    this->m_thermalFault[sub] = faultNow;

    const U8 dT = atLeastOne(this->m_debounceThermal);
    if (this->m_thermalCount[sub] >= dT && !this->isFaulted(sub)) {
        this->sendFault(sub, Billee::FaultReason::OVERTEMP);
    }
}

void FPManager ::powerStateIn_handler(FwIndexType portNum,
                                      const Billee::Subsystems& subsystem,
                                      const Fw::On& state) {
    (void)portNum;
    this->applyPendingPowerStates();  // keep ordering: anything parked earlier goes first
    U8 sub = 0;
    if (!subIndexFor(subsystem, sub) || sub == SUB_LOGIC) {
        return;
    }
    this->applyPowerState(sub, state == Fw::On::ON);
}

void FPManager ::powerStateIn_overflowHook(FwIndexType portNum,
                                           const Billee::Subsystems& subsystem,
                                           const Fw::On& state) {
    (void)portNum;
    U8 sub = 0;
    if (!subIndexFor(subsystem, sub) || sub == SUB_LOGIC) {
        return;
    }
    // Runs on SubsystemManager's thread: park the latest state; FPManager applies it before
    // evaluating its next message.
    this->m_pendingPowerState[sub] = (state == Fw::On::ON) ? 2 : 1;
    this->m_droppedReadings = this->m_droppedReadings + 1;
    this->tlmWrite_FpReadingsDropped(this->m_droppedReadings);
}

void FPManager ::applyPowerState(const U8 sub, const bool on) {
    this->m_commandedOn[sub] = on;
    if (on) {
        this->m_onSince[sub] = this->nowSeconds();
    }
    // Undervoltage evidence from before this change is meaningless either way
    for (U8 i = 0; i < NUM_INA; i++) {
        if (this->m_sensor[i].subIndex == sub) {
            this->m_sensor[i].uv = 0;
        }
    }
}

void FPManager ::applyPendingPowerStates() {
    for (U8 sub = 0; sub < SUB_LOGIC; sub++) {
        const U8 pending = this->m_pendingPowerState[sub];
        if (pending != 0) {
            this->m_pendingPowerState[sub] = 0;
            this->applyPowerState(sub, pending == 2);
        }
    }
}

void FPManager ::powerReadingIn_overflowHook(FwIndexType portNum,
                                             const Billee::Subsystems& subsystem,
                                             const Billee::PowerReading& reading) {
    (void)portNum;
    (void)subsystem;
    (void)reading;
    this->m_droppedReadings = this->m_droppedReadings + 1;
    this->tlmWrite_FpReadingsDropped(this->m_droppedReadings);
    if (!this->m_dropReported) {
        this->m_dropReported = true;
        this->log_WARNING_HI_FpReadingDropped();
    }
}

void FPManager ::thermalReadingIn_overflowHook(FwIndexType portNum,
                                               const Billee::Subsystems& subsystem,
                                               const Billee::ThermalReading& reading) {
    (void)portNum;
    (void)subsystem;
    (void)reading;
    this->m_droppedReadings = this->m_droppedReadings + 1;
    this->tlmWrite_FpReadingsDropped(this->m_droppedReadings);
    if (!this->m_dropReported) {
        this->m_dropReported = true;
        this->log_WARNING_HI_FpReadingDropped();
    }
}

// ----------------------------------------------------------------------
// Command handlers
// ----------------------------------------------------------------------

void FPManager ::CLEAR_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Billee::Subsystems subsystem) {
    U8 sub = 0;
    if (!subIndexFor(subsystem, sub)) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    if (!this->isFaulted(sub)) {
        this->log_ACTIVITY_LO_FaultClearNoop(subsystem);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
        return;
    }
    if (this->m_thermalFault[sub]) {
        this->log_WARNING_LO_FaultClearRejected(subsystem);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    // Clearing is allowed while the E-STOP is engaged: it only releases the inhibit.
    this->sendClear(sub);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Parameters
// ----------------------------------------------------------------------

void FPManager ::parameterUpdated(FwPrmIdType id) {
    (void)id;
    this->loadParams();
}

void FPManager ::loadParamsIfNeeded() {
    if (!this->m_paramsLoaded) {
        this->loadParams();
    }
}

void FPManager ::loadParams() {
    this->m_paramsLoaded = true;
    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    this->m_bounds[SUB_DRIVETRAIN] = this->paramGet_DRIVETRAIN_BOUNDS(valid);
    this->m_bounds[SUB_ARM] = this->paramGet_ARM_BOUNDS(valid);
    this->m_bounds[SUB_SCIENCE] = this->paramGet_SCIENCE_BOUNDS(valid);
    this->m_bounds[SUB_LOGIC] = this->paramGet_LOGIC_BOUNDS(valid);
    this->m_uvSettleS = this->paramGet_UV_SETTLE_S(valid);
    this->m_debounceOc = this->paramGet_DEBOUNCE_OC(valid);
    this->m_debounceOv = this->paramGet_DEBOUNCE_OV(valid);
    this->m_debounceUv = this->paramGet_DEBOUNCE_UV(valid);
    this->m_debounceThermal = this->paramGet_DEBOUNCE_THERMAL(valid);
    this->publishThresholdTelemetry();
}

// ----------------------------------------------------------------------
// State machine actions
// ----------------------------------------------------------------------

void FPManager ::Billee_FPStateMachine_action_doTrip(SmId smId,
                                                     Billee_FPStateMachine::Signal signal,
                                                     const Billee::FaultReason& value) {
    (void)signal;
    const U8 sub = subIndexFor(smId);
    const Billee::Subsystems subsystem = subsystemFor(sub);
    if (isControllable(sub)) {
        if (this->isConnected_faultInhibitOut_OutputPort(0)) {
            this->faultInhibitOut_out(0, subsystem, true);
        }
        this->log_WARNING_HI_SubsystemFaultShutdown(subsystem, value);
    } else {
        this->log_WARNING_HI_LogicFaultDetected(value);
    }
    this->writeFaultStateTelemetry(sub, Billee::FaultState::TRIPPED);
}

void FPManager ::Billee_FPStateMachine_action_doClear(SmId smId, Billee_FPStateMachine::Signal signal) {
    (void)signal;
    const U8 sub = subIndexFor(smId);
    const Billee::Subsystems subsystem = subsystemFor(sub);
    this->resetSubsystemCounters(sub);
    if (isControllable(sub)) {
        if (this->isConnected_faultInhibitOut_OutputPort(0)) {
            this->faultInhibitOut_out(0, subsystem, false);
        }
        this->log_ACTIVITY_HI_SubsystemFaultCleared(subsystem);
    } else {
        this->log_ACTIVITY_HI_LogicFaultCleared();
    }
    this->writeFaultStateTelemetry(sub, Billee::FaultState::NOMINAL);
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

bool FPManager ::subIndexFor(const Billee::Subsystems subsystem, U8& index) {
    switch (subsystem.e) {
        case Billee::Subsystems::DRIVETRAIN:
            index = SUB_DRIVETRAIN;
            return true;
        case Billee::Subsystems::ARM:
            index = SUB_ARM;
            return true;
        case Billee::Subsystems::SCIENCE:
            index = SUB_SCIENCE;
            return true;
        case Billee::Subsystems::LOGIC:
            index = SUB_LOGIC;
            return true;
        default:
            return false;  // AUX: no hardware
    }
}

U8 FPManager ::subIndexFor(const SmId smId) {
    switch (smId) {
        case SmId::fp_drivetrainSM:
            return SUB_DRIVETRAIN;
        case SmId::fp_armSM:
            return SUB_ARM;
        case SmId::fp_scienceSM:
            return SUB_SCIENCE;
        case SmId::fp_logicSM:
        default:
            return SUB_LOGIC;
    }
}

Billee::Subsystems FPManager ::subsystemFor(const U8 subIndex) {
    switch (subIndex) {
        case SUB_DRIVETRAIN:
            return Billee::Subsystems::DRIVETRAIN;
        case SUB_ARM:
            return Billee::Subsystems::ARM;
        case SUB_SCIENCE:
            return Billee::Subsystems::SCIENCE;
        case SUB_LOGIC:
        default:
            return Billee::Subsystems::LOGIC;
    }
}

bool FPManager ::isControllable(const U8 subIndex) {
    return subIndex != SUB_LOGIC;
}

bool FPManager ::isFaulted(const U8 subIndex) {
    switch (subIndex) {
        case SUB_DRIVETRAIN:
            return this->fp_drivetrainSM_getState() == Billee_FPStateMachine::State::FAULTED;
        case SUB_ARM:
            return this->fp_armSM_getState() == Billee_FPStateMachine::State::FAULTED;
        case SUB_SCIENCE:
            return this->fp_scienceSM_getState() == Billee_FPStateMachine::State::FAULTED;
        case SUB_LOGIC:
        default:
            return this->fp_logicSM_getState() == Billee_FPStateMachine::State::FAULTED;
    }
}

void FPManager ::sendFault(const U8 subIndex, const Billee::FaultReason reason) {
    switch (subIndex) {
        case SUB_DRIVETRAIN:
            this->fp_drivetrainSM_sendSignal_fault(reason);
            break;
        case SUB_ARM:
            this->fp_armSM_sendSignal_fault(reason);
            break;
        case SUB_SCIENCE:
            this->fp_scienceSM_sendSignal_fault(reason);
            break;
        case SUB_LOGIC:
        default:
            this->fp_logicSM_sendSignal_fault(reason);
            break;
    }
}

void FPManager ::sendClear(const U8 subIndex) {
    switch (subIndex) {
        case SUB_DRIVETRAIN:
            this->fp_drivetrainSM_sendSignal_clearRequest();
            break;
        case SUB_ARM:
            this->fp_armSM_sendSignal_clearRequest();
            break;
        case SUB_SCIENCE:
            this->fp_scienceSM_sendSignal_clearRequest();
            break;
        case SUB_LOGIC:
        default:
            this->fp_logicSM_sendSignal_clearRequest();
            break;
    }
}

void FPManager ::resetSubsystemCounters(const U8 subIndex) {
    for (U8 i = 0; i < NUM_INA; i++) {
        if (this->m_sensor[i].subIndex == subIndex) {
            this->m_sensor[i].oc = 0;
            this->m_sensor[i].ov = 0;
            this->m_sensor[i].uv = 0;
        }
    }
    this->m_thermalCount[subIndex] = 0;
}

F64 FPManager ::nowSeconds() {
    const Fw::Time now = this->getTime();
    return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

void FPManager ::writeFaultStateTelemetry(const U8 subIndex, const Billee::FaultState state) {
    switch (subIndex) {
        case SUB_DRIVETRAIN:
            this->tlmWrite_DRIVETRAIN_FAULT_STATE(state);
            break;
        case SUB_ARM:
            this->tlmWrite_ARM_FAULT_STATE(state);
            break;
        case SUB_SCIENCE:
            this->tlmWrite_SCIENCE_FAULT_STATE(state);
            break;
        case SUB_LOGIC:
        default:
            this->tlmWrite_LOGIC_FAULT_STATE(state);
            break;
    }
}

void FPManager ::writeSensorStateTelemetry(const Billee::InaSensorId sensorId, const Billee::FaultState state) {
    switch (sensorId.e) {
        case Billee::InaSensorId::DRIVE1:
            this->tlmWrite_DRIVE1_POWER_STATE(state);
            break;
        case Billee::InaSensorId::DRIVE2:
            this->tlmWrite_DRIVE2_POWER_STATE(state);
            break;
        case Billee::InaSensorId::DRIVE3:
            this->tlmWrite_DRIVE3_POWER_STATE(state);
            break;
        case Billee::InaSensorId::DRIVE4:
            this->tlmWrite_DRIVE4_POWER_STATE(state);
            break;
        case Billee::InaSensorId::DRIVE5:
            this->tlmWrite_DRIVE5_POWER_STATE(state);
            break;
        case Billee::InaSensorId::DRIVE6:
            this->tlmWrite_DRIVE6_POWER_STATE(state);
            break;
        case Billee::InaSensorId::ARM:
            this->tlmWrite_ARM_POWER_STATE(state);
            break;
        case Billee::InaSensorId::SCIENCE:
            this->tlmWrite_SCIENCE_POWER_STATE(state);
            break;
        case Billee::InaSensorId::LOGIC:
            this->tlmWrite_LOGIC_POWER_STATE(state);
            break;
        default:
            break;
    }
}

void FPManager ::publishThresholdTelemetry() {
    this->tlmWrite_DRIVETRAIN_POWER_BOUNDS(this->m_bounds[SUB_DRIVETRAIN]);
    this->tlmWrite_ARM_POWER_BOUNDS(this->m_bounds[SUB_ARM]);
    this->tlmWrite_SCIENCE_POWER_BOUNDS(this->m_bounds[SUB_SCIENCE]);
    this->tlmWrite_LOGIC_POWER_BOUNDS(this->m_bounds[SUB_LOGIC]);
}

}  // namespace Billee
