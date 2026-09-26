// ======================================================================
// \title  SubsystemManager.cpp
// \author luquito
// \brief  SubsystemManager component implementation
// ======================================================================
#include "Components/SubsystemManager/SubsystemManager.hpp"

namespace Billee {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

SubsystemManager::SubsystemManager(const char* const compName) : SubsystemManagerComponentBase(compName) {}

SubsystemManager::~SubsystemManager() {}

// ----------------------------------------------------------------------
// Static helpers
// ----------------------------------------------------------------------

Fw::Logic SubsystemManager::toLogic(const Fw::On state) {
    return state == Fw::On::ON ? Fw::Logic::HIGH : Fw::Logic::LOW;
}

bool SubsystemManager::gpioOpSucceeded(const Drv::GpioStatus status) {
    return status == Drv::GpioStatus::OP_OK;
}

bool SubsystemManager::toIndex(const Billee::Subsystems subsystem, U8& index) {
    switch (subsystem.e) {
        case Billee::Subsystems::DRIVETRAIN:
            index = IDX_DRIVETRAIN;
            return true;
        case Billee::Subsystems::ARM:
            index = IDX_ARM;
            return true;
        case Billee::Subsystems::SCIENCE:
            index = IDX_SCIENCE;
            return true;
        default:
            // AUX has no hardware; LOGIC is the flight computer itself. Neither is controllable.
            return false;
    }
}

Billee::Subsystems SubsystemManager::fromIndex(const U8 index) {
    switch (index) {
        case IDX_DRIVETRAIN:
            return Billee::Subsystems::DRIVETRAIN;
        case IDX_ARM:
            return Billee::Subsystems::ARM;
        case IDX_SCIENCE:
        default:
            return Billee::Subsystems::SCIENCE;
    }
}

// ----------------------------------------------------------------------
// Private helpers
// ----------------------------------------------------------------------

SubsystemManager::EStopState SubsystemManager::readEStop() {
    Fw::Logic level = Fw::Logic::LOW;
    const Drv::GpioStatus status = this->EStopRead_out(0, level);
    if (!gpioOpSucceeded(status)) {
        return EStopState::UNKNOWN;
    }
    // Hardware: E-STOP switch pulls the line to 3.3 V when released; R61 pulls it LOW when pressed.
    return (level == Fw::Logic::HIGH) ? EStopState::RELEASED : EStopState::ENGAGED;
}

bool SubsystemManager::writeGpios(const Billee::Subsystems subsystem, const Fw::Logic level) {
    switch (subsystem.e) {
        case Billee::Subsystems::DRIVETRAIN: {
            // Execute every write even if an earlier one fails, so the six enables stay consistent.
            const bool ok1 = gpioOpSucceeded(this->Drive1Set_out(0, level));
            const bool ok2 = gpioOpSucceeded(this->Drive2Set_out(0, level));
            const bool ok3 = gpioOpSucceeded(this->Drive3Set_out(0, level));
            const bool ok4 = gpioOpSucceeded(this->Drive4Set_out(0, level));
            const bool ok5 = gpioOpSucceeded(this->Drive5Set_out(0, level));
            const bool ok6 = gpioOpSucceeded(this->Drive6Set_out(0, level));
            return ok1 && ok2 && ok3 && ok4 && ok5 && ok6;
        }
        case Billee::Subsystems::ARM:
            return gpioOpSucceeded(this->ArmSet_out(0, level));
        case Billee::Subsystems::SCIENCE:
            return gpioOpSucceeded(this->ScienceSet_out(0, level));
        default:
            return false;
    }
}

void SubsystemManager::writeStateTelemetry(const Billee::Subsystems subsystem) {
    U8 index = 0;
    if (!toIndex(subsystem, index)) {
        return;
    }
    switch (subsystem.e) {
        case Billee::Subsystems::DRIVETRAIN:
            this->tlmWrite_DrivetrainPowerState(this->m_state[index]);
            break;
        case Billee::Subsystems::ARM:
            this->tlmWrite_ArmPowerState(this->m_state[index]);
            break;
        case Billee::Subsystems::SCIENCE:
            this->tlmWrite_SciencePowerState(this->m_state[index]);
            break;
        default:
            break;
    }
}

bool SubsystemManager::applyState(const Billee::Subsystems subsystem, const Fw::On state) {
    U8 index = 0;
    if (!toIndex(subsystem, index)) {
        return false;
    }

    bool ok = this->writeGpios(subsystem, toLogic(state));
    Fw::On newState = state;
    if (!ok) {
        // Fail safe: whatever was requested, drive every enable of this subsystem LOW.
        (void)this->writeGpios(subsystem, Fw::Logic::LOW);
        newState = Fw::On::OFF;
        this->log_WARNING_HI_PowerWriteFailed(subsystem);
    }

    if (newState != this->m_state[index]) {
        this->m_state[index] = newState;
        this->log_ACTIVITY_HI_SubsystemPowerModeEvent(subsystem, newState);
        this->writeStateTelemetry(subsystem);
        if (this->isConnected_powerStateOut_OutputPort(0)) {
            this->powerStateOut_out(0, subsystem, newState);
        }
    }
    return ok;
}

bool SubsystemManager::forceAllOff() {
    bool anyWasOn = false;
    for (U8 index = 0; index < NUM_CONTROLLED; index++) {
        if (this->m_state[index] == Fw::On::ON) {
            anyWasOn = true;
            (void)this->applyState(fromIndex(index), Fw::On::OFF);
        }
    }
    return anyWasOn;
}

void SubsystemManager::handlePowerCommand(const FwOpcodeType opCode,
                                          const U32 cmdSeq,
                                          const Billee::Subsystems subsystem,
                                          const Fw::On requested) {
    U8 index = 0;
    if (!toIndex(subsystem, index)) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }

    if (requested == Fw::On::ON) {
        // Fresh read: never trust the cached value for a power-ON decision.
        const EStopState eStop = this->readEStop();
        if (eStop != EStopState::RELEASED) {
            const Billee::PowerOnRejectReason reason = (eStop == EStopState::ENGAGED)
                                                           ? Billee::PowerOnRejectReason::E_STOP_ENGAGED
                                                           : Billee::PowerOnRejectReason::E_STOP_UNKNOWN;
            this->log_WARNING_LO_PowerOnRejected(subsystem, reason);
            this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
            return;
        }
        if (this->m_inhibited[index]) {
            this->log_WARNING_LO_PowerOnRejected(subsystem, Billee::PowerOnRejectReason::FAULT_LATCHED);
            this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
            return;
        }
    }

    // OFF is always executed, even when the E-STOP is engaged or the subsystem is inhibited.
    const bool ok = this->applyState(subsystem, requested);
    this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void SubsystemManager::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    const EStopState eStop = this->readEStop();

    if (eStop == EStopState::UNKNOWN) {
        if (!this->m_eStopReadFailed) {
            this->m_eStopReadFailed = true;
            this->log_WARNING_HI_EStopReadFailure();
        }
    } else {
        this->m_eStopReadFailed = false;
    }

    if (eStop != this->m_eStop) {
        if (eStop == EStopState::ENGAGED) {
            this->log_WARNING_HI_EStopEngaged();
        } else if (eStop == EStopState::RELEASED) {
            this->log_ACTIVITY_HI_EStopReleased();
        }
    }
    this->m_eStop = eStop;

    // Engaged or unreadable: nothing may stay ON. Release restores nothing.
    if (eStop != EStopState::RELEASED) {
        if (this->forceAllOff()) {
            this->log_WARNING_HI_EStopForcedOff();
        }
    }

    for (U8 index = 0; index < NUM_CONTROLLED; index++) {
        this->writeStateTelemetry(fromIndex(index));
    }
    this->tlmWrite_E_STOP_Status((eStop == EStopState::RELEASED) ? Fw::On::OFF : Fw::On::ON);
}

void SubsystemManager::faultInhibitIn_handler(FwIndexType portNum,
                                              const Billee::Subsystems& subsystem,
                                              bool inhibit) {
    (void)portNum;
    U8 index = 0;
    if (!toIndex(subsystem, index)) {
        return;
    }
    if (inhibit) {
        this->m_inhibited[index] = true;
        (void)this->applyState(subsystem, Fw::On::OFF);
        this->log_WARNING_HI_FaultInhibitSet(subsystem);
    } else {
        // Releasing an inhibit never turns anything on.
        this->m_inhibited[index] = false;
        this->log_ACTIVITY_HI_FaultInhibitCleared(subsystem);
    }
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void SubsystemManager::SET_DRIVETRAIN_POWER_STATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Fw::On driveState) {
    this->handlePowerCommand(opCode, cmdSeq, Billee::Subsystems::DRIVETRAIN, driveState);
}

void SubsystemManager::SET_ARM_POWER_STATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Fw::On armState) {
    this->handlePowerCommand(opCode, cmdSeq, Billee::Subsystems::ARM, armState);
}

void SubsystemManager::SET_SCIENCE_POWER_STATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Fw::On scienceState) {
    this->handlePowerCommand(opCode, cmdSeq, Billee::Subsystems::SCIENCE, scienceState);
}

}  // namespace Billee
