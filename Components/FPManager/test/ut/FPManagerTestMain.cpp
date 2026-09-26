// ======================================================================
// \title  FPManagerTestMain.cpp
// \brief  FPManager unit tests (ids F1-F12 match the firmware fix plan)
// ======================================================================

#include "FPManagerTester.hpp"

#define FP_TEST(name, method)            \
    TEST(FPManager, name) {              \
        Billee::FPManagerTester tester;  \
        tester.method();                 \
    }

FP_TEST(F1_OffChannelAtZeroVoltsNeverTrips, testF1_offChannelAtZeroVoltsNeverTrips)
FP_TEST(F2_FourCellDriveVoltageIsNominal, testF2_fourCellDriveVoltageIsNominal)
FP_TEST(F3_UndervoltageGatedBySettleAndDebounced, testF3_undervoltageGatedBySettleAndDebounced)
FP_TEST(F4_OvercurrentTripsWholeDrivetrain, testF4_overcurrentTripsWholeDrivetrain)
FP_TEST(F4b_OvercurrentThresholdFollowsParameter, testF4b_overcurrentThresholdFollowsParameter)
FP_TEST(F5_OtherWheelsDoNotClearLatchedFault, testF5_otherWheelsDoNotClearLatchedFault)
FP_TEST(F6_ClearFaultReleasesInhibit, testF6_clearFaultReleasesInhibit)
FP_TEST(F7_ClearRejectedWhileOverTemperature, testF7_clearRejectedWhileOverTemperature)
FP_TEST(F8_InvalidReadingsAreNotEvaluated, testF8_invalidReadingsAreNotEvaluated)
FP_TEST(F9_ArmBoundsAndUndervoltage, testF9_armBoundsAndUndervoltage)
FP_TEST(F10_ThermalFaultTripsFailureIsAlertOnly, testF10_thermalFaultTripsFailureIsAlertOnly)
FP_TEST(F11_LogicFaultIsAlertOnly, testF11_logicFaultIsAlertOnly)
FP_TEST(F12_CommandedOffStopsUndervoltageCheck, testF12_commandedOffStopsUndervoltageCheck)
FP_TEST(F13_ParkedPowerStateIsApplied, testF13_parkedPowerStateIsApplied)

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
