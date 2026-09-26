// ======================================================================
// \title  FPManagerTester.cpp
// \brief  cpp file for FPManager component test harness implementation class
// ======================================================================

#include "FPManagerTester.hpp"

#include "Fw/Types/String.hpp"

namespace Billee {

namespace {
Billee::Subsystems subsystemOf(Billee::InaSensorId sensor) {
    switch (sensor.e) {
        case Billee::InaSensorId::ARM:
            return Billee::Subsystems::ARM;
        case Billee::InaSensorId::SCIENCE:
            return Billee::Subsystems::SCIENCE;
        case Billee::InaSensorId::LOGIC:
            return Billee::Subsystems::LOGIC;
        default:
            return Billee::Subsystems::DRIVETRAIN;
    }
}
const Billee::InaSensorId::T kWheels[6] = {Billee::InaSensorId::DRIVE1, Billee::InaSensorId::DRIVE2,
                                           Billee::InaSensorId::DRIVE3, Billee::InaSensorId::DRIVE4,
                                           Billee::InaSensorId::DRIVE5, Billee::InaSensorId::DRIVE6};
}  // namespace

// ----------------------------------------------------------------------
// Construction and destruction
// ----------------------------------------------------------------------

FPManagerTester ::FPManagerTester()
    : FPManagerGTestBase("FPManagerTester", FPManagerTester::MAX_HISTORY_SIZE), component("FPManager") {
    this->initComponents();
    this->connectPorts();
    this->component.loadParameters();  // no stored params: every param uses its fpp default
    this->at(1000.0);
}

FPManagerTester ::~FPManagerTester() {
    this->component.deinit();
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

void FPManagerTester ::drain() {
    // Process queued port calls and the state-machine signals they generate
    while (this->component.m_queue.getMessagesAvailable() > 0) {
        this->component.doDispatch();
    }
}

void FPManagerTester ::at(F64 seconds) {
    const U32 s = static_cast<U32>(seconds);
    const U32 us = static_cast<U32>((seconds - static_cast<F64>(s)) * 1e6 + 0.5);
    this->setTestTime(Fw::Time(s, us));
}

void FPManagerTester ::powerState(Billee::Subsystems subsystem, Fw::On state) {
    this->invoke_to_powerStateIn(0, subsystem, state);
    this->drain();
}

void FPManagerTester ::reading(Billee::InaSensorId sensor, F32 volts, F32 amps, bool valid) {
    Billee::PowerReading r;
    r.set_voltage(volts);
    r.set_current(amps);
    r.set_power(volts * amps);
    r.set_sourceId(sensor);
    r.set_timestamp(0);
    r.set_valid(valid);
    this->invoke_to_powerReadingIn(0, subsystemOf(sensor), r);
    this->drain();
}

void FPManagerTester ::allWheels(F32 volts, F32 amps, U32 samples) {
    for (U32 n = 0; n < samples; n++) {
        for (const Billee::InaSensorId::T w : kWheels) {
            this->reading(w, volts, amps);
        }
    }
}

void FPManagerTester ::thermal(Billee::Subsystems subsystem, Billee::McpSensorId sensor, Billee::ThermalStates state) {
    Billee::ThermalReading t;
    t.set_temperature(25.0f);
    t.set_tempState(state);
    t.set_sensorId(sensor);
    t.set_location(Fw::String("ut"));
    t.set_timestamp(0);
    this->invoke_to_thermalReadingIn(0, subsystem, t);
    this->drain();
}

void FPManagerTester ::clearFault(Billee::Subsystems subsystem, U32 cmdSeq) {
    this->sendCmd_CLEAR_FAULT(0, cmdSeq, subsystem);
    this->drain();
}

bool FPManagerTester ::faulted(Billee::Subsystems subsystem) {
    U8 sub = 0;
    if (!FPManager::subIndexFor(subsystem, sub)) {
        return false;
    }
    return this->component.isFaulted(sub);
}

// ----------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------

// F1: a commanded-OFF drivetrain reads ~0 V on its switched-side VBUS. That must never trip.
void FPManagerTester ::testF1_offChannelAtZeroVoltsNeverTrips() {
    this->allWheels(0.0f, 0.0f, 20);
    ASSERT_from_faultInhibitOut_SIZE(0);
    ASSERT_EVENTS_SubsystemFaultShutdown_SIZE(0);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::DRIVETRAIN));
}

// F2: 4S voltages are nominal for the drivetrain (the old global 6S bound tripped here)
void FPManagerTester ::testF2_fourCellDriveVoltageIsNominal() {
    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::ON);
    this->at(1002.0);
    this->allWheels(14.8f, 2.0f, 10);
    this->allWheels(13.7f, 4.0f, 10);
    ASSERT_from_faultInhibitOut_SIZE(0);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::DRIVETRAIN));
}

// F3: undervoltage is ignored during the settle window, then needs 5 consecutive samples
void FPManagerTester ::testF3_undervoltageGatedBySettleAndDebounced() {
    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::ON);  // at t=1000.0
    this->at(1000.5);
    for (U32 n = 0; n < 10; n++) {
        this->reading(Billee::InaSensorId::DRIVE1, 13.4f, 1.0f);
    }
    ASSERT_from_faultInhibitOut_SIZE(0);

    this->at(1001.5);
    for (U32 n = 0; n < 4; n++) {
        this->reading(Billee::InaSensorId::DRIVE1, 13.4f, 1.0f);
    }
    ASSERT_from_faultInhibitOut_SIZE(0);
    this->reading(Billee::InaSensorId::DRIVE1, 13.4f, 1.0f);
    ASSERT_from_faultInhibitOut_SIZE(1);
    ASSERT_from_faultInhibitOut(0, Billee::Subsystems::DRIVETRAIN, true);
    ASSERT_EVENTS_SubsystemFaultShutdown(0, Billee::Subsystems::DRIVETRAIN, Billee::FaultReason::UNDERVOLTAGE);
    ASSERT_TRUE(this->faulted(Billee::Subsystems::DRIVETRAIN));
}

// F4: overcurrent on one wheel (default 30 A) trips the whole drivetrain, once
void FPManagerTester ::testF4_overcurrentTripsWholeDrivetrain() {
    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::ON);
    this->at(1002.0);
    for (U32 n = 0; n < 10; n++) {
        this->reading(Billee::InaSensorId::DRIVE4, 14.8f, 29.9f);
    }
    ASSERT_from_faultInhibitOut_SIZE(0);

    for (U32 n = 0; n < 3; n++) {
        this->reading(Billee::InaSensorId::DRIVE4, 14.8f, 31.0f);
    }
    ASSERT_from_faultInhibitOut_SIZE(1);
    ASSERT_from_faultInhibitOut(0, Billee::Subsystems::DRIVETRAIN, true);
    ASSERT_EVENTS_PowerFaultDetail_SIZE(1);
    ASSERT_EVENTS_SubsystemFaultShutdown(0, Billee::Subsystems::DRIVETRAIN, Billee::FaultReason::OVERCURRENT);

    // Negative current beyond the limit counts too (applied as +/-)
    ASSERT_TRUE(this->faulted(Billee::Subsystems::DRIVETRAIN));
}

// F4b: the overcurrent threshold follows DRIVETRAIN_BOUNDS when changed by parameter
void FPManagerTester ::testF4b_overcurrentThresholdFollowsParameter() {
    Billee::PowerBounds b(13.6f, 17.2f, 5.0f);
    this->paramSet_DRIVETRAIN_BOUNDS(b, Fw::ParamValid::VALID);
    this->paramSend_DRIVETRAIN_BOUNDS(0, 0);
    this->drain();

    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::ON);
    this->at(1002.0);
    for (U32 n = 0; n < 5; n++) {
        this->reading(Billee::InaSensorId::DRIVE2, 14.8f, 4.9f);
    }
    ASSERT_from_faultInhibitOut_SIZE(0);
    for (U32 n = 0; n < 3; n++) {
        this->reading(Billee::InaSensorId::DRIVE2, 14.8f, -5.1f);
    }
    ASSERT_from_faultInhibitOut_SIZE(1);
    ASSERT_EVENTS_SubsystemFaultShutdown(0, Billee::Subsystems::DRIVETRAIN, Billee::FaultReason::OVERCURRENT);
}

// F5: after a trip, nominal readings from the other wheels don't clear it, and nothing re-trips
void FPManagerTester ::testF5_otherWheelsDoNotClearLatchedFault() {
    this->testF4_overcurrentTripsWholeDrivetrain();
    this->clearHistory();
    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::OFF);  // SubsystemManager forced it off
    this->allWheels(14.8f, 0.0f, 10);
    this->allWheels(0.0f, 0.0f, 10);
    for (U32 n = 0; n < 5; n++) {
        this->reading(Billee::InaSensorId::DRIVE4, 14.8f, 31.0f);
    }
    ASSERT_TRUE(this->faulted(Billee::Subsystems::DRIVETRAIN));
    ASSERT_from_faultInhibitOut_SIZE(0);  // no release, no second inhibit
    ASSERT_EVENTS_SubsystemFaultShutdown_SIZE(0);
    ASSERT_EVENTS_SubsystemFaultCleared_SIZE(0);
}

// F6: CLEAR_FAULT releases the inhibit and returns to NOMINAL
void FPManagerTester ::testF6_clearFaultReleasesInhibit() {
    this->testF4_overcurrentTripsWholeDrivetrain();
    this->clearHistory();
    this->clearFault(Billee::Subsystems::DRIVETRAIN, 42);
    ASSERT_CMD_RESPONSE(0, FPManager::OPCODE_CLEAR_FAULT, 42, Fw::CmdResponse::OK);
    ASSERT_from_faultInhibitOut_SIZE(1);
    ASSERT_from_faultInhibitOut(0, Billee::Subsystems::DRIVETRAIN, false);
    ASSERT_EVENTS_SubsystemFaultCleared(0, Billee::Subsystems::DRIVETRAIN);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::DRIVETRAIN));

    // Clearing again is a no-op
    this->clearHistory();
    this->clearFault(Billee::Subsystems::DRIVETRAIN, 43);
    ASSERT_CMD_RESPONSE(0, FPManager::OPCODE_CLEAR_FAULT, 43, Fw::CmdResponse::OK);
    ASSERT_EVENTS_FaultClearNoop_SIZE(1);
    ASSERT_from_faultInhibitOut_SIZE(0);

    // Detection works again after the clear
    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::ON);
    this->at(1010.0);
    for (U32 n = 0; n < 3; n++) {
        this->reading(Billee::InaSensorId::DRIVE1, 14.8f, 31.0f);
    }
    ASSERT_TRUE(this->faulted(Billee::Subsystems::DRIVETRAIN));
}

// F7: CLEAR_FAULT is refused while the subsystem's temperature is still at FAULT level
void FPManagerTester ::testF7_clearRejectedWhileOverTemperature() {
    this->thermal(Billee::Subsystems::DRIVETRAIN, Billee::McpSensorId::DRIVE_TEMP, Billee::ThermalStates::FAULT);
    this->thermal(Billee::Subsystems::DRIVETRAIN, Billee::McpSensorId::DRIVE_TEMP, Billee::ThermalStates::FAULT);
    ASSERT_TRUE(this->faulted(Billee::Subsystems::DRIVETRAIN));
    this->clearHistory();

    this->clearFault(Billee::Subsystems::DRIVETRAIN, 7);
    ASSERT_CMD_RESPONSE(0, FPManager::OPCODE_CLEAR_FAULT, 7, Fw::CmdResponse::EXECUTION_ERROR);
    ASSERT_EVENTS_FaultClearRejected(0, Billee::Subsystems::DRIVETRAIN);
    ASSERT_TRUE(this->faulted(Billee::Subsystems::DRIVETRAIN));
    ASSERT_from_faultInhibitOut_SIZE(0);

    // Once the temperature is back below FAULT, the clear is accepted
    this->thermal(Billee::Subsystems::DRIVETRAIN, Billee::McpSensorId::DRIVE_TEMP, Billee::ThermalStates::WARN);
    this->clearFault(Billee::Subsystems::DRIVETRAIN, 8);
    ASSERT_CMD_RESPONSE(1, FPManager::OPCODE_CLEAR_FAULT, 8, Fw::CmdResponse::OK);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::DRIVETRAIN));
}

// F8: invalid (failed-read) readings are never evaluated: 0 V while ON and settled doesn't trip
void FPManagerTester ::testF8_invalidReadingsAreNotEvaluated() {
    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::ON);
    this->at(1005.0);
    this->clearHistory();
    for (U32 n = 0; n < 20; n++) {
        this->reading(Billee::InaSensorId::DRIVE2, 0.0f, 0.0f, false);
    }
    ASSERT_from_faultInhibitOut_SIZE(0);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::DRIVETRAIN));
    ASSERT_TLM_DRIVE2_POWER_STATE_SIZE(20);
    ASSERT_TLM_DRIVE2_POWER_STATE(19, Billee::FaultState::STALE);
}

// F9: arm uses its own 6S bounds
void FPManagerTester ::testF9_armBoundsAndUndervoltage() {
    this->powerState(Billee::Subsystems::ARM, Fw::On::ON);
    this->at(1002.0);
    for (U32 n = 0; n < 10; n++) {
        this->reading(Billee::InaSensorId::ARM, 22.2f, 3.0f);
    }
    ASSERT_from_faultInhibitOut_SIZE(0);
    for (U32 n = 0; n < 4; n++) {
        this->reading(Billee::InaSensorId::ARM, 21.4f, 3.0f);
    }
    ASSERT_from_faultInhibitOut_SIZE(0);
    this->reading(Billee::InaSensorId::ARM, 21.4f, 3.0f);
    ASSERT_from_faultInhibitOut(0, Billee::Subsystems::ARM, true);
    ASSERT_EVENTS_SubsystemFaultShutdown(0, Billee::Subsystems::ARM, Billee::FaultReason::UNDERVOLTAGE);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::DRIVETRAIN));
}

// F10: a real over-temperature (2 samples) trips; an unreadable sensor is alert-only
void FPManagerTester ::testF10_thermalFaultTripsFailureIsAlertOnly() {
    // FAILURE on the shared arm/science sensor: one event, no trip on either subsystem
    for (U32 n = 0; n < 5; n++) {
        this->thermal(Billee::Subsystems::ARM, Billee::McpSensorId::ARM_SCI_TEMP, Billee::ThermalStates::FAILURE);
        this->thermal(Billee::Subsystems::SCIENCE, Billee::McpSensorId::ARM_SCI_TEMP, Billee::ThermalStates::FAILURE);
    }
    ASSERT_EVENTS_ThermalSensorLost_SIZE(1);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::ARM));
    ASSERT_FALSE(this->faulted(Billee::Subsystems::SCIENCE));
    ASSERT_from_faultInhibitOut_SIZE(0);

    // Recovery is reported once; a single FAULT sample doesn't trip, the second does
    this->thermal(Billee::Subsystems::SCIENCE, Billee::McpSensorId::ARM_SCI_TEMP, Billee::ThermalStates::FAULT);
    ASSERT_EVENTS_ThermalSensorRecovered_SIZE(1);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::SCIENCE));
    this->thermal(Billee::Subsystems::SCIENCE, Billee::McpSensorId::ARM_SCI_TEMP, Billee::ThermalStates::FAULT);
    ASSERT_TRUE(this->faulted(Billee::Subsystems::SCIENCE));
    ASSERT_EVENTS_SubsystemFaultShutdown(0, Billee::Subsystems::SCIENCE, Billee::FaultReason::OVERTEMP);
    ASSERT_from_faultInhibitOut(0, Billee::Subsystems::SCIENCE, true);
}

// F11: logic faults are alert-only and never inhibit anything
void FPManagerTester ::testF11_logicFaultIsAlertOnly() {
    for (U32 n = 0; n < 3; n++) {
        this->reading(Billee::InaSensorId::LOGIC, 22.2f, 31.0f);
    }
    ASSERT_TRUE(this->faulted(Billee::Subsystems::LOGIC));
    ASSERT_EVENTS_LogicFaultDetected_SIZE(1);
    ASSERT_from_faultInhibitOut_SIZE(0);

    // Logic undervoltage is checked without any power-state gating (it measures its own battery)
    FPManagerTester fresh;
    for (U32 n = 0; n < 5; n++) {
        fresh.reading(Billee::InaSensorId::LOGIC, 21.0f, 2.0f);
    }
    ASSERT_TRUE(fresh.faulted(Billee::Subsystems::LOGIC));
}

// F12: when a subsystem is commanded OFF (e.g. the E-STOP forced it), its 0 V readings don't trip
void FPManagerTester ::testF12_commandedOffStopsUndervoltageCheck() {
    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::ON);
    this->at(1003.0);
    this->allWheels(14.8f, 3.0f, 3);
    // E-STOP pressed: hardware cuts power, readings fall to 0 V a couple of samples before
    // SubsystemManager reports OFF (up to one 10 Hz poll later)
    this->allWheels(0.0f, 0.0f, 2);
    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::OFF);
    this->allWheels(0.0f, 0.0f, 20);
    ASSERT_from_faultInhibitOut_SIZE(0);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::DRIVETRAIN));
}

// F13: a power-state change parked by the queue-overflow hook is applied before the next reading
void FPManagerTester ::testF13_parkedPowerStateIsApplied() {
    this->powerState(Billee::Subsystems::DRIVETRAIN, Fw::On::ON);
    this->at(1003.0);
    // Simulate SubsystemManager's OFF notification arriving while the queue was full
    this->component.powerStateIn_overflowHook(0, Billee::Subsystems::DRIVETRAIN, Fw::On::OFF);
    this->allWheels(0.0f, 0.0f, 20);
    ASSERT_from_faultInhibitOut_SIZE(0);
    ASSERT_FALSE(this->faulted(Billee::Subsystems::DRIVETRAIN));
    ASSERT_TLM_FpReadingsDropped_SIZE(1);
}

}  // namespace Billee
