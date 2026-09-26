// ======================================================================
// \title  FPManagerTester.hpp
// \brief  hpp file for FPManager component test harness implementation class
// ======================================================================

#ifndef Billee_FPManagerTester_HPP
#define Billee_FPManagerTester_HPP

#include "Components/FPManager/FPManager.hpp"
#include "Components/FPManager/FPManagerGTestBase.hpp"

namespace Billee {

class FPManagerTester final : public FPManagerGTestBase {
  public:
    static const FwSizeType MAX_HISTORY_SIZE = 256;
    static const FwEnumStoreType TEST_INSTANCE_ID = 0;
    static const FwSizeType TEST_INSTANCE_QUEUE_DEPTH = 64;

    FPManagerTester();
    ~FPManagerTester();

    // Tests (ids match the firmware fix plan)
    void testF1_offChannelAtZeroVoltsNeverTrips();
    void testF2_fourCellDriveVoltageIsNominal();
    void testF3_undervoltageGatedBySettleAndDebounced();
    void testF4_overcurrentTripsWholeDrivetrain();
    void testF4b_overcurrentThresholdFollowsParameter();
    void testF5_otherWheelsDoNotClearLatchedFault();
    void testF6_clearFaultReleasesInhibit();
    void testF7_clearRejectedWhileOverTemperature();
    void testF8_invalidReadingsAreNotEvaluated();
    void testF9_armBoundsAndUndervoltage();
    void testF10_thermalFaultTripsFailureIsAlertOnly();
    void testF11_logicFaultIsAlertOnly();
    void testF12_commandedOffStopsUndervoltageCheck();
    void testF13_parkedPowerStateIsApplied();

  private:
    void connectPorts();
    void initComponents();

    // Helpers
    void at(F64 seconds);
    void powerState(Billee::Subsystems subsystem, Fw::On state);
    void reading(Billee::InaSensorId sensor, F32 volts, F32 amps, bool valid = true);
    void allWheels(F32 volts, F32 amps, U32 samples);
    void thermal(Billee::Subsystems subsystem, Billee::McpSensorId sensor, Billee::ThermalStates state);
    void clearFault(Billee::Subsystems subsystem, U32 cmdSeq);
    bool faulted(Billee::Subsystems subsystem);
    void drain();

    FPManager component;
};

}  // namespace Billee

#endif
