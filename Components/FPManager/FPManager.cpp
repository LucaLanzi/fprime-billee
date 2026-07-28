// ======================================================================
// \title  FPManager.cpp
// \brief  cpp file for FPManager component implementation class
// ======================================================================

#include "Components/FPManager/FPManager.hpp"

namespace Billee {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

FPManager ::FPManager(const char* const compName) : FPManagerComponentBase(compName) {}

FPManager ::~FPManager() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void FPManager ::powerReadingIn_handler(FwIndexType portNum,
                                        const Billee::Subsystems& subsystem,
                                        const Billee::PowerReading& reading) {
    this->loadParamsIfNeeded();
    this->writePowerSensorStateTelemetry(reading);
    // Republished every reading (not just once at first load) so a GDS client connecting after
    // boot still sees the current bounds without waiting for a fault trip/clear or param change.
    this->publishThresholdTelemetry();
    switch (subsystem) {
        case Billee::Subsystems::DRIVETRAIN:
            this->fp_drivetrainSM_sendSignal_powerUpdate(reading);
            break;
        case Billee::Subsystems::ARM:
            this->fp_armSM_sendSignal_powerUpdate(reading);
            break;
        case Billee::Subsystems::SCIENCE:
            this->fp_scienceSM_sendSignal_powerUpdate(reading);
            break;
        case Billee::Subsystems::LOGIC:
            this->fp_logicSM_sendSignal_powerUpdate(reading);
            break;
        default:
            // AUX has no power/thermal sensor coverage, so FPManager cannot protect it.
            break;
    }
}

void FPManager ::thermalReadingIn_handler(FwIndexType portNum,
                                          const Billee::Subsystems& subsystem,
                                          const Billee::ThermalReading& reading) {
    this->loadParamsIfNeeded();
    switch (subsystem) {
        case Billee::Subsystems::DRIVETRAIN:
            this->fp_drivetrainSM_sendSignal_thermalUpdate(reading);
            break;
        case Billee::Subsystems::ARM:
            this->fp_armSM_sendSignal_thermalUpdate(reading);
            break;
        case Billee::Subsystems::SCIENCE:
            this->fp_scienceSM_sendSignal_thermalUpdate(reading);
            break;
        case Billee::Subsystems::LOGIC:
            this->fp_logicSM_sendSignal_thermalUpdate(reading);
            break;
        default:
            break;
    }
}

// ----------------------------------------------------------------------
// Handler implementation for parameter updates
// ----------------------------------------------------------------------

void FPManager ::parameterUpdated(FwPrmIdType id) {
    switch (id) {
        case PARAMID_VBUS_FAULT_LOW:
            this->m_vbusFaultLow = this->paramGet_VBUS_FAULT_LOW(this->m_paramIsValid);
            break;
        case PARAMID_VBUS_FAULT_HIGH:
            this->m_vbusFaultHigh = this->paramGet_VBUS_FAULT_HIGH(this->m_paramIsValid);
            break;
        case PARAMID_CURRENT_FAULT_HIGH:
            this->m_currentFaultHigh = this->paramGet_CURRENT_FAULT_HIGH(this->m_paramIsValid);
            break;
        default:
            break;
    }
    this->publishThresholdTelemetry();
}

// ----------------------------------------------------------------------
// Implementations for internal state machine guards
// ----------------------------------------------------------------------

bool FPManager ::Billee_FPStateMachine_guard_isPowerFault(SmId smId,
                                                          Billee_FPStateMachine::Signal signal,
                                                          const Billee::PowerReading& data) const {
    return this->isPowerReadingOutOfBounds(data) || this->cacheFor(smId).thermalFaulted;
}

bool FPManager ::Billee_FPStateMachine_guard_isThermalFault(SmId smId,
                                                            Billee_FPStateMachine::Signal signal,
                                                            const Billee::ThermalReading& data) const {
    const Billee::ThermalStates state = data.get_tempState();
    const bool fromData =
        (state == Billee::ThermalStates::FAULT) || (state == Billee::ThermalStates::FAILURE);
    return fromData || this->cacheFor(smId).powerFaulted;
}

// ----------------------------------------------------------------------
// Implementations for internal state machine actions
// ----------------------------------------------------------------------

void FPManager ::Billee_FPStateMachine_action_doTripFromPower(SmId smId,
                                                              Billee_FPStateMachine::Signal signal,
                                                              const Billee::PowerReading& data) {
    this->cacheFor(smId).powerFaulted = true;

    Billee::FaultReason reason = Billee::FaultReason::OVERCURRENT;
    const F32 voltage = data.get_voltage();
    if (voltage < this->m_vbusFaultLow) {
        reason = Billee::FaultReason::UNDERVOLTAGE;
    } else if (voltage > this->m_vbusFaultHigh) {
        reason = Billee::FaultReason::OVERVOLTAGE;
    }

    const Billee::Subsystems subsystem = FPManager::subsystemFor(smId);
    if (FPManager::isControllable(smId)) {
        this->emergencyPowerOffOut_out(0, subsystem, Fw::On::OFF);
        this->log_WARNING_HI_SubsystemFaultShutdown(subsystem, reason);
    } else {
        this->log_WARNING_HI_LogicFaultDetected(reason);
    }

    this->writeFaultStateTelemetry(smId, Billee::FaultState::TRIPPED);
    this->publishThresholdTelemetry();
}

void FPManager ::Billee_FPStateMachine_action_doTripFromThermal(SmId smId,
                                                                Billee_FPStateMachine::Signal signal,
                                                                const Billee::ThermalReading& data) {
    this->cacheFor(smId).thermalFaulted = true;

    const Billee::FaultReason reason = (data.get_tempState() == Billee::ThermalStates::FAILURE)
                                            ? Billee::FaultReason::SENSOR_FAILURE
                                            : Billee::FaultReason::OVERTEMP;

    const Billee::Subsystems subsystem = FPManager::subsystemFor(smId);
    if (FPManager::isControllable(smId)) {
        this->emergencyPowerOffOut_out(0, subsystem, Fw::On::OFF);
        this->log_WARNING_HI_SubsystemFaultShutdown(subsystem, reason);
    } else {
        this->log_WARNING_HI_LogicFaultDetected(reason);
    }

    this->writeFaultStateTelemetry(smId, Billee::FaultState::TRIPPED);
    this->publishThresholdTelemetry();
}

void FPManager ::Billee_FPStateMachine_action_doClear(SmId smId, Billee_FPStateMachine::Signal signal) {
    this->cacheFor(smId).powerFaulted = false;
    this->cacheFor(smId).thermalFaulted = false;

    if (FPManager::isControllable(smId)) {
        this->log_ACTIVITY_HI_SubsystemFaultCleared(FPManager::subsystemFor(smId));
    } else {
        this->log_ACTIVITY_HI_LogicFaultCleared();
    }

    this->writeFaultStateTelemetry(smId, Billee::FaultState::NOMINAL);
    this->publishThresholdTelemetry();
}

// ----------------------------------------------------------------------
// Helper functions
// ----------------------------------------------------------------------

FPManager::FaultCache& FPManager ::cacheFor(SmId smId) {
    return const_cast<FaultCache&>(static_cast<const FPManager&>(*this).cacheFor(smId));
}

const FPManager::FaultCache& FPManager ::cacheFor(SmId smId) const {
    switch (smId) {
        case SmId::fp_drivetrainSM:
            return this->m_drivetrainCache;
        case SmId::fp_armSM:
            return this->m_armCache;
        case SmId::fp_scienceSM:
            return this->m_scienceCache;
        case SmId::fp_logicSM:
        default:
            return this->m_logicCache;
    }
}

Billee::Subsystems FPManager ::subsystemFor(SmId smId) {
    switch (smId) {
        case SmId::fp_drivetrainSM:
            return Billee::Subsystems::DRIVETRAIN;
        case SmId::fp_armSM:
            return Billee::Subsystems::ARM;
        case SmId::fp_scienceSM:
            return Billee::Subsystems::SCIENCE;
        case SmId::fp_logicSM:
        default:
            return Billee::Subsystems::LOGIC;
    }
}

bool FPManager ::isControllable(SmId smId) {
    return smId != SmId::fp_logicSM;
}

void FPManager ::writeFaultStateTelemetry(SmId smId, Billee::FaultState state) {
    switch (smId) {
        case SmId::fp_drivetrainSM:
            this->tlmWrite_DRIVETRAIN_FAULT_STATE(state);
            break;
        case SmId::fp_armSM:
            this->tlmWrite_ARM_FAULT_STATE(state);
            break;
        case SmId::fp_scienceSM:
            this->tlmWrite_SCIENCE_FAULT_STATE(state);
            break;
        case SmId::fp_logicSM:
            this->tlmWrite_LOGIC_FAULT_STATE(state);
            break;
    }
}

void FPManager ::loadParamsIfNeeded() {
    if (this->m_paramsLoaded) {
        return;
    }
    this->m_paramsLoaded = true;

    this->m_vbusFaultLow = this->paramGet_VBUS_FAULT_LOW(this->m_paramIsValid);
    this->m_vbusFaultHigh = this->paramGet_VBUS_FAULT_HIGH(this->m_paramIsValid);
    this->m_currentFaultHigh = this->paramGet_CURRENT_FAULT_HIGH(this->m_paramIsValid);
    this->publishThresholdTelemetry();
}

void FPManager ::publishThresholdTelemetry() {
    Billee::PowerBounds bounds;
    bounds.set_vbusFaultLow(this->m_vbusFaultLow);
    bounds.set_vbusFaultHigh(this->m_vbusFaultHigh);
    bounds.set_currentFaultHigh(this->m_currentFaultHigh);
    // All four subsystems currently share the same global threshold params (see FPManager.fpp).
    this->tlmWrite_DRIVETRAIN_POWER_BOUNDS(bounds);
    this->tlmWrite_ARM_POWER_BOUNDS(bounds);
    this->tlmWrite_SCIENCE_POWER_BOUNDS(bounds);
    this->tlmWrite_LOGIC_POWER_BOUNDS(bounds);
}

bool FPManager ::isPowerReadingOutOfBounds(const Billee::PowerReading& reading) const {
    const F32 voltage = reading.get_voltage();
    const F32 current = reading.get_current();
    return (voltage < this->m_vbusFaultLow) || (voltage > this->m_vbusFaultHigh) ||
           (current > this->m_currentFaultHigh) || (current < -this->m_currentFaultHigh);
}

void FPManager ::writePowerSensorStateTelemetry(const Billee::PowerReading& reading) {
    const Billee::FaultState state =
        this->isPowerReadingOutOfBounds(reading) ? Billee::FaultState::TRIPPED : Billee::FaultState::NOMINAL;

    switch (reading.get_sourceId()) {
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
    }
}

}  // namespace Billee
