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

bool SubsystemManager::gpioWriteSucceeded(
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
        gpioWriteSucceeded(drive1Status) &&
        gpioWriteSucceeded(drive2Status) &&
        gpioWriteSucceeded(drive3Status) &&
        gpioWriteSucceeded(drive4Status) &&
        gpioWriteSucceeded(drive5Status) &&
        gpioWriteSucceeded(drive6Status);
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

    this->tlmWrite_AuxPowerState(
        this->m_auxState
    );
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

    if (!this->gpioWriteSucceeded(writeStatus)) {
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

    if (!this->gpioWriteSucceeded(writeStatus)) {
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

}  // namespace Billee