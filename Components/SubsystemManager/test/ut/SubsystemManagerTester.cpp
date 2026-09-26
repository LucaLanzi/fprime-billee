// ======================================================================
// \title  SubsystemManagerTester.cpp
// \brief  cpp file for SubsystemManager component test harness implementation class
// ======================================================================

#include "SubsystemManagerTester.hpp"

namespace Billee {

// ----------------------------------------------------------------------
// Construction and destruction
// ----------------------------------------------------------------------

SubsystemManagerTester ::SubsystemManagerTester()
    : SubsystemManagerGTestBase("SubsystemManagerTester", SubsystemManagerTester::MAX_HISTORY_SIZE),
      component("SubsystemManager") {
    for (U8 i = 0; i < NUM_PINS; i++) {
        this->m_pin[i] = Fw::Logic::LOW;
    }
    this->initComponents();
    this->connectPorts();
}

SubsystemManagerTester ::~SubsystemManagerTester() {
    this->component.deinit();
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

void SubsystemManagerTester ::runOnce() {
    this->invoke_to_run(0, 0);
    this->component.doDispatch();
}

void SubsystemManagerTester ::command(Billee::Subsystems subsystem, Fw::On state, U32 cmdSeq) {
    switch (subsystem.e) {
        case Billee::Subsystems::DRIVETRAIN:
            this->sendCmd_SET_DRIVETRAIN_POWER_STATE(0, cmdSeq, state);
            break;
        case Billee::Subsystems::ARM:
            this->sendCmd_SET_ARM_POWER_STATE(0, cmdSeq, state);
            break;
        default:
            this->sendCmd_SET_SCIENCE_POWER_STATE(0, cmdSeq, state);
            break;
    }
    this->component.doDispatch();
}

bool SubsystemManagerTester ::allDriveLow() const {
    for (U8 i = 0; i < 6; i++) {
        if (this->m_pin[i] != Fw::Logic::LOW) {
            return false;
        }
    }
    return true;
}

bool SubsystemManagerTester ::allDriveHigh() const {
    for (U8 i = 0; i < 6; i++) {
        if (this->m_pin[i] != Fw::Logic::HIGH) {
            return false;
        }
    }
    return true;
}

Drv::GpioStatus SubsystemManagerTester ::writePin(U8 pin, const Fw::Logic& state) {
    this->m_pin[pin] = state;
    return Drv::GpioStatus::OP_OK;
}

// Simulated GPIO drivers (record the level, then let the base class record history)
Drv::GpioStatus SubsystemManagerTester ::from_Drive1Set_handler(FwIndexType portNum, const Fw::Logic& state) {
    SubsystemManagerGTestBase::from_Drive1Set_handler(portNum, state);
    return this->writePin(0, state);
}
Drv::GpioStatus SubsystemManagerTester ::from_Drive2Set_handler(FwIndexType portNum, const Fw::Logic& state) {
    SubsystemManagerGTestBase::from_Drive2Set_handler(portNum, state);
    return this->writePin(1, state);
}
Drv::GpioStatus SubsystemManagerTester ::from_Drive3Set_handler(FwIndexType portNum, const Fw::Logic& state) {
    SubsystemManagerGTestBase::from_Drive3Set_handler(portNum, state);
    return this->writePin(2, state);
}
Drv::GpioStatus SubsystemManagerTester ::from_Drive4Set_handler(FwIndexType portNum, const Fw::Logic& state) {
    SubsystemManagerGTestBase::from_Drive4Set_handler(portNum, state);
    if (this->m_failDrive4 && state == Fw::Logic::HIGH) {
        return Drv::GpioStatus::UNKNOWN_ERROR;  // pin does not change
    }
    return this->writePin(3, state);
}
Drv::GpioStatus SubsystemManagerTester ::from_Drive5Set_handler(FwIndexType portNum, const Fw::Logic& state) {
    SubsystemManagerGTestBase::from_Drive5Set_handler(portNum, state);
    return this->writePin(4, state);
}
Drv::GpioStatus SubsystemManagerTester ::from_Drive6Set_handler(FwIndexType portNum, const Fw::Logic& state) {
    SubsystemManagerGTestBase::from_Drive6Set_handler(portNum, state);
    return this->writePin(5, state);
}
Drv::GpioStatus SubsystemManagerTester ::from_ArmSet_handler(FwIndexType portNum, const Fw::Logic& state) {
    SubsystemManagerGTestBase::from_ArmSet_handler(portNum, state);
    return this->writePin(PIN_ARM, state);
}
Drv::GpioStatus SubsystemManagerTester ::from_ScienceSet_handler(FwIndexType portNum, const Fw::Logic& state) {
    SubsystemManagerGTestBase::from_ScienceSet_handler(portNum, state);
    return this->writePin(PIN_SCIENCE, state);
}
Drv::GpioStatus SubsystemManagerTester ::from_EStopRead_handler(FwIndexType portNum, Fw::Logic& state) {
    SubsystemManagerGTestBase::from_EStopRead_handler(portNum, state);
    if (this->m_eStopFail) {
        return Drv::GpioStatus::UNKNOWN_ERROR;
    }
    state = this->m_eStopLevel;
    return Drv::GpioStatus::OP_OK;
}

// ----------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------

// S1: ON while the E-STOP reads LOW is rejected and no enable goes HIGH
void SubsystemManagerTester ::testS1_onRejectedWhileEStopEngaged() {
    this->setEStopEngaged();
    this->runOnce();
    this->clearHistory();

    this->command(Billee::Subsystems::ARM, Fw::On::ON, 1);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, SubsystemManager::OPCODE_SET_ARM_POWER_STATE, 1, Fw::CmdResponse::VALIDATION_ERROR);
    ASSERT_EVENTS_PowerOnRejected_SIZE(1);
    ASSERT_EVENTS_PowerOnRejected(0, Billee::Subsystems::ARM, Billee::PowerOnRejectReason::E_STOP_ENGAGED);
    ASSERT_from_ArmSet_SIZE(0);
    ASSERT_from_powerStateOut_SIZE(0);
    ASSERT_EQ(this->m_pin[PIN_ARM], Fw::Logic::LOW);
}

// S2: ON while the E-STOP line can't be read is rejected (treated as engaged)
void SubsystemManagerTester ::testS2_onRejectedWhenEStopUnreadable() {
    this->m_eStopFail = true;
    this->runOnce();
    ASSERT_EVENTS_EStopReadFailure_SIZE(1);
    this->clearHistory();

    this->command(Billee::Subsystems::DRIVETRAIN, Fw::On::ON, 2);
    ASSERT_CMD_RESPONSE(0, SubsystemManager::OPCODE_SET_DRIVETRAIN_POWER_STATE, 2, Fw::CmdResponse::VALIDATION_ERROR);
    ASSERT_EVENTS_PowerOnRejected(0, Billee::Subsystems::DRIVETRAIN, Billee::PowerOnRejectReason::E_STOP_UNKNOWN);
    ASSERT_TRUE(this->allDriveLow());

    // Repeated failures don't spam the event
    this->runOnce();
    ASSERT_EVENTS_EStopReadFailure_SIZE(0);
}

// S3: ON with the E-STOP released drives the pins HIGH and notifies FPManager
void SubsystemManagerTester ::testS3_onAcceptedWhenReleased() {
    this->setEStopReleased();
    this->runOnce();
    this->clearHistory();

    this->command(Billee::Subsystems::DRIVETRAIN, Fw::On::ON, 3);
    ASSERT_CMD_RESPONSE(0, SubsystemManager::OPCODE_SET_DRIVETRAIN_POWER_STATE, 3, Fw::CmdResponse::OK);
    ASSERT_TRUE(this->allDriveHigh());
    ASSERT_from_powerStateOut_SIZE(1);
    ASSERT_from_powerStateOut(0, Billee::Subsystems::DRIVETRAIN, Fw::On::ON);
    ASSERT_EVENTS_SubsystemPowerModeEvent(0, Billee::Subsystems::DRIVETRAIN, Fw::On::ON);

    // Repeating ON is idempotent: no second notification
    this->command(Billee::Subsystems::DRIVETRAIN, Fw::On::ON, 4);
    ASSERT_from_powerStateOut_SIZE(1);
}

// S4: engaging the E-STOP forces everything OFF within one run; releasing restores nothing
void SubsystemManagerTester ::testS4_eStopForcesOffAndReleaseRestoresNothing() {
    this->setEStopReleased();
    this->runOnce();
    this->command(Billee::Subsystems::ARM, Fw::On::ON, 5);
    this->command(Billee::Subsystems::DRIVETRAIN, Fw::On::ON, 6);
    ASSERT_TRUE(this->allDriveHigh());
    ASSERT_EQ(this->m_pin[PIN_ARM], Fw::Logic::HIGH);
    this->clearHistory();

    this->setEStopEngaged();
    this->runOnce();
    ASSERT_TRUE(this->allDriveLow());
    ASSERT_EQ(this->m_pin[PIN_ARM], Fw::Logic::LOW);
    ASSERT_EVENTS_EStopEngaged_SIZE(1);
    ASSERT_EVENTS_EStopForcedOff_SIZE(1);
    ASSERT_from_powerStateOut_SIZE(2);
    ASSERT_TLM_E_STOP_Status(0, Fw::On::ON);
    this->clearHistory();

    this->setEStopReleased();
    this->runOnce();
    this->runOnce();
    ASSERT_EVENTS_EStopReleased_SIZE(1);
    ASSERT_TRUE(this->allDriveLow());
    ASSERT_EQ(this->m_pin[PIN_ARM], Fw::Logic::LOW);
    ASSERT_from_powerStateOut_SIZE(0);
    ASSERT_EVENTS_EStopForcedOff_SIZE(0);
}

// S5: a fault inhibit forces OFF and blocks ON until released; release turns nothing on
void SubsystemManagerTester ::testS5_faultInhibitBlocksOn() {
    this->setEStopReleased();
    this->runOnce();
    this->command(Billee::Subsystems::ARM, Fw::On::ON, 7);
    ASSERT_EQ(this->m_pin[PIN_ARM], Fw::Logic::HIGH);
    this->clearHistory();

    this->invoke_to_faultInhibitIn(0, Billee::Subsystems::ARM, true);
    this->component.doDispatch();
    ASSERT_EQ(this->m_pin[PIN_ARM], Fw::Logic::LOW);
    ASSERT_EVENTS_FaultInhibitSet(0, Billee::Subsystems::ARM);
    ASSERT_from_powerStateOut(0, Billee::Subsystems::ARM, Fw::On::OFF);
    this->clearHistory();

    this->command(Billee::Subsystems::ARM, Fw::On::ON, 8);
    ASSERT_CMD_RESPONSE(0, SubsystemManager::OPCODE_SET_ARM_POWER_STATE, 8, Fw::CmdResponse::VALIDATION_ERROR);
    ASSERT_EVENTS_PowerOnRejected(0, Billee::Subsystems::ARM, Billee::PowerOnRejectReason::FAULT_LATCHED);
    ASSERT_EQ(this->m_pin[PIN_ARM], Fw::Logic::LOW);
    // Other subsystems are unaffected
    this->command(Billee::Subsystems::SCIENCE, Fw::On::ON, 9);
    ASSERT_EQ(this->m_pin[PIN_SCIENCE], Fw::Logic::HIGH);
    this->clearHistory();

    this->invoke_to_faultInhibitIn(0, Billee::Subsystems::ARM, false);
    this->component.doDispatch();
    ASSERT_EVENTS_FaultInhibitCleared(0, Billee::Subsystems::ARM);
    ASSERT_EQ(this->m_pin[PIN_ARM], Fw::Logic::LOW);  // release never turns anything on
    ASSERT_from_powerStateOut_SIZE(0);

    this->command(Billee::Subsystems::ARM, Fw::On::ON, 10);
    ASSERT_CMD_RESPONSE(0, SubsystemManager::OPCODE_SET_ARM_POWER_STATE, 10, Fw::CmdResponse::OK);
    ASSERT_EQ(this->m_pin[PIN_ARM], Fw::Logic::HIGH);
}

// S6: if one drivetrain write fails, all six enables are driven LOW
void SubsystemManagerTester ::testS6_partialDrivetrainWriteFailureDrivesAllLow() {
    this->setEStopReleased();
    this->runOnce();
    this->m_failDrive4 = true;
    this->clearHistory();

    this->command(Billee::Subsystems::DRIVETRAIN, Fw::On::ON, 11);
    ASSERT_CMD_RESPONSE(0, SubsystemManager::OPCODE_SET_DRIVETRAIN_POWER_STATE, 11, Fw::CmdResponse::EXECUTION_ERROR);
    ASSERT_TRUE(this->allDriveLow());
    ASSERT_EVENTS_PowerWriteFailed(0, Billee::Subsystems::DRIVETRAIN);
    ASSERT_from_powerStateOut_SIZE(0);  // it never became ON
}

// S7: OFF is always accepted, even when the E-STOP is engaged and the subsystem is inhibited
void SubsystemManagerTester ::testS7_offAlwaysAccepted() {
    this->setEStopEngaged();
    this->runOnce();
    this->invoke_to_faultInhibitIn(0, Billee::Subsystems::SCIENCE, true);
    this->component.doDispatch();
    this->clearHistory();

    this->command(Billee::Subsystems::SCIENCE, Fw::On::OFF, 12);
    ASSERT_CMD_RESPONSE(0, SubsystemManager::OPCODE_SET_SCIENCE_POWER_STATE, 12, Fw::CmdResponse::OK);
    ASSERT_EQ(this->m_pin[PIN_SCIENCE], Fw::Logic::LOW);
    ASSERT_from_ScienceSet_SIZE(1);
}

// S8: AUX is gone. This is a compile-level check: the component has exactly these three commands.
void SubsystemManagerTester ::testS8_noAuxInterface() {
    ASSERT_EQ(static_cast<U32>(SubsystemManager::OPCODE_SET_DRIVETRAIN_POWER_STATE), 0u);
    ASSERT_EQ(static_cast<U32>(SubsystemManager::OPCODE_SET_ARM_POWER_STATE), 1u);
    ASSERT_EQ(static_cast<U32>(SubsystemManager::OPCODE_SET_SCIENCE_POWER_STATE), 3u);
    // (No OPCODE_SET_AUX_POWER_STATE, AuxSet port or AuxPowerState channel exists: referencing
    // any of them here would fail to compile.)
}

}  // namespace Billee
