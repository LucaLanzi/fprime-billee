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

SubsystemManager::SubsystemManager(const char* const compName)
    : SubsystemManagerComponentBase(compName) {}

SubsystemManager::~SubsystemManager() {}
// ----------------------------------------------------------------------
// Private helper functions
// ----------------------------------------------------------------------

Fw::Logic SubsystemManager::toLogic(const Fw::On state) {
    return state == Fw::On::ON
               ? Fw::Logic::HIGH
               : Fw::Logic::LOW;
}

bool SubsystemManager::gpioOpSucceeded(
    const Drv::GpioStatus status
) {
    return status == Drv::GpioStatus::OP_OK;
}
bool SubsystemManager::setDrivetrainGpios(
    const Fw::Logic state
) {
    // Execute every write, even if an earlier write fails. This keeps the
    // six drivetrain enable outputs as consistent as possible.
    const Drv::GpioStatus drive1Status =
        this->Drive1Set_out(0, state);

    const Drv::GpioStatus drive2Status =
        this->Drive2Set_out(0, state);

    const Drv::GpioStatus drive3Status =
        this->Drive3Set_out(0, state);
    const Drv::GpioStatus drive4Status =
        this->Drive4Set_out(0, state);

    const Drv::GpioStatus drive5Status =
        this->Drive5Set_out(0, state);

    const Drv::GpioStatus drive6Status =
        this->Drive6Set_out(0, state);
    return
        gpioOpSucceeded(drive1Status) &&
        gpioOpSucceeded(drive2Status) &&
        gpioOpSucceeded(drive3Status) &&
        gpioOpSucceeded(drive4Status) &&
        gpioOpSucceeded(drive5Status) &&
        gpioOpSucceeded(drive6Status);
}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------
void SubsystemManager::run_handler(
    FwIndexType portNum,
    U32 context
) {
    (void)portNum;
    (void)context;

    // Publish the component's commanded GPIO states.
    this->tlmWrite_DrivetrainPowerState(
        this->m_drivetrainState
    );

    this->tlmWrite_ArmPowerState(
        this->m_armState
    );

    this->tlmWrite_SciencePowerState(
        this->m_scienceState
    );

    this->tlmWrite_AuxPowerState(
        this->m_auxState
    );

    // Poll the E-STOP status input. LOW = on, HIGH = off (see EStopRead port doc).
    Fw::Logic eStopLogicState = Fw::Logic::HIGH;
    const Drv::GpioStatus eStopReadStatus = this->EStopRead_out(0, eStopLogicState);

    if (this->gpioOpSucceeded(eStopReadStatus)) {
        const Fw::On newEStopState = (eStopLogicState == Fw::Logic::LOW) ? Fw::On::ON : Fw::On::OFF;

        if (newEStopState != this->m_eStopState) {
            if (newEStopState == Fw::On::ON) {
                this->log_WARNING_HI_EStopEngaged();
            } else {
                this->log_ACTIVITY_HI_EStopReleased();
            }
        }

        this->m_eStopState = newEStopState;
    }

    this->tlmWrite_E_STOP_Status(
        this->m_eStopState
    );
}
void SubsystemManager::emergencyPowerOff_handler(
    FwIndexType portNum,
    const Billee::Subsystems& subsystem,
    const Fw::On& state
) {
    (void)portNum;

    switch (subsystem) {
        case Billee::Subsystems::DRIVETRAIN: {
            if (this->setDrivetrainGpios(this->toLogic(state)) && state != this->m_drivetrainState) {
                this->m_drivetrainState = state;
                this->log_ACTIVITY_HI_SubsystemPowerModeEvent(Billee::Subsystems::DRIVETRAIN, state);
                this->tlmWrite_DrivetrainPowerState(this->m_drivetrainState);
            }
            break;
        }
        case Billee::Subsystems::ARM: {
            const Drv::GpioStatus writeStatus = this->ArmSet_out(0, this->toLogic(state));
            if (this->gpioOpSucceeded(writeStatus) && state != this->m_armState) {
                this->m_armState = state;
                this->log_ACTIVITY_HI_SubsystemPowerModeEvent(Billee::Subsystems::ARM, state);
                this->tlmWrite_ArmPowerState(this->m_armState);
            }
            break;
        }
        case Billee::Subsystems::SCIENCE: {
            const Drv::GpioStatus writeStatus = this->ScienceSet_out(0, this->toLogic(state));
            if (this->gpioOpSucceeded(writeStatus) && state != this->m_scienceState) {
                this->m_scienceState = state;
                this->log_ACTIVITY_HI_SubsystemPowerModeEvent(Billee::Subsystems::SCIENCE, state);
                this->tlmWrite_SciencePowerState(this->m_scienceState);
            }
            break;
        }
        case Billee::Subsystems::AUX: {
            const Drv::GpioStatus writeStatus = this->AuxSet_out(0, this->toLogic(state));
            if (this->gpioOpSucceeded(writeStatus) && state != this->m_auxState) {
                this->m_auxState = state;
                this->log_ACTIVITY_HI_SubsystemPowerModeEvent(Billee::Subsystems::AUX, state);
                this->tlmWrite_AuxPowerState(this->m_auxState);
            }
            break;
        }
        default:
            // LOGIC has no power control (it is the flight computer itself); ignore.
            break;
    }
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void SubsystemManager::SET_DRIVETRAIN_POWER_STATE_cmdHandler(
    FwOpcodeType opCode,
    U32 cmdSeq,
    Fw::On driveState
) {
    const bool writeSuccessful =
        this->setDrivetrainGpios(
            this->toLogic(driveState)
        );
    if (!writeSuccessful) {
        this->cmdResponse_out(
            opCode,
            cmdSeq,
            Fw::CmdResponse::EXECUTION_ERROR
        );
        return;
    }

    if (driveState != this->m_drivetrainState) {
        this->m_drivetrainState = driveState;

        this->log_ACTIVITY_HI_SubsystemPowerModeEvent(
            Billee::Subsystems::DRIVETRAIN,
            driveState
        );
        // Optional immediate telemetry update. The run handler will also
        // periodically publish the state.
        this->tlmWrite_DrivetrainPowerState(
            this->m_drivetrainState
        );
    }

    this->cmdResponse_out(
        opCode,
        cmdSeq,
        Fw::CmdResponse::OK
    );
}
void SubsystemManager::SET_ARM_POWER_STATE_cmdHandler(
    FwOpcodeType opCode,
    U32 cmdSeq,
    Fw::On armState
) {
    const Drv::GpioStatus writeStatus =
        this->ArmSet_out(
            0,
            this->toLogic(armState)
        );

    if (!this->gpioOpSucceeded(writeStatus)) {
        this->cmdResponse_out(
            opCode,
            cmdSeq,
            Fw::CmdResponse::EXECUTION_ERROR
        );
        return;
    }
    if (armState != this->m_armState) {
        this->m_armState = armState;

        this->log_ACTIVITY_HI_SubsystemPowerModeEvent(
            Billee::Subsystems::ARM,
            armState
        );

        this->tlmWrite_ArmPowerState(
            this->m_armState
        );
    }

    this->cmdResponse_out(
        opCode,
        cmdSeq,
        Fw::CmdResponse::OK
    );
}
void SubsystemManager::SET_AUX_POWER_STATE_cmdHandler(
    FwOpcodeType opCode,
    U32 cmdSeq,
    Fw::On auxState
) {
    const Drv::GpioStatus writeStatus =
        this->AuxSet_out(
            0,
            this->toLogic(auxState)
        );

    if (!this->gpioOpSucceeded(writeStatus)) {
        this->cmdResponse_out(
            opCode,
            cmdSeq,
            Fw::CmdResponse::EXECUTION_ERROR
        );
        return;
    }
    if (auxState != this->m_auxState) {
        this->m_auxState = auxState;

        this->log_ACTIVITY_HI_SubsystemPowerModeEvent(
            Billee::Subsystems::AUX,
            auxState
        );

        this->tlmWrite_AuxPowerState(
            this->m_auxState
        );
    }

    this->cmdResponse_out(
        opCode,
        cmdSeq,
        Fw::CmdResponse::OK
    );
}
void SubsystemManager::SET_SCIENCE_POWER_STATE_cmdHandler(
    FwOpcodeType opCode,
    U32 cmdSeq,
    Fw::On scienceState
) {
    const Drv::GpioStatus writeStatus =
        this->ScienceSet_out(
            0,
            this->toLogic(scienceState)
        );

    if (!this->gpioOpSucceeded(writeStatus)) {
        this->cmdResponse_out(
            opCode,
            cmdSeq,
            Fw::CmdResponse::EXECUTION_ERROR
        );
        return;
    }
    if (scienceState != this->m_scienceState) {
        this->m_scienceState = scienceState;

        this->log_ACTIVITY_HI_SubsystemPowerModeEvent(
            Billee::Subsystems::SCIENCE,
            scienceState
        );

        this->tlmWrite_SciencePowerState(
            this->m_scienceState
        );
    }

    this->cmdResponse_out(
        opCode,
        cmdSeq,
        Fw::CmdResponse::OK
    );
}

}  // namespace Billee
