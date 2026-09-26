// ======================================================================
// \title  SubsystemManagerTestMain.cpp
// \brief  SubsystemManager unit tests (ids S1-S8 match the firmware fix plan)
// ======================================================================

#include "SubsystemManagerTester.hpp"

TEST(SubsystemManager, S1_OnRejectedWhileEStopEngaged) {
    Billee::SubsystemManagerTester tester;
    tester.testS1_onRejectedWhileEStopEngaged();
}

TEST(SubsystemManager, S2_OnRejectedWhenEStopUnreadable) {
    Billee::SubsystemManagerTester tester;
    tester.testS2_onRejectedWhenEStopUnreadable();
}

TEST(SubsystemManager, S3_OnAcceptedWhenReleased) {
    Billee::SubsystemManagerTester tester;
    tester.testS3_onAcceptedWhenReleased();
}

TEST(SubsystemManager, S4_EStopForcesOffAndReleaseRestoresNothing) {
    Billee::SubsystemManagerTester tester;
    tester.testS4_eStopForcesOffAndReleaseRestoresNothing();
}

TEST(SubsystemManager, S5_FaultInhibitBlocksOn) {
    Billee::SubsystemManagerTester tester;
    tester.testS5_faultInhibitBlocksOn();
}

TEST(SubsystemManager, S6_PartialDrivetrainWriteFailureDrivesAllLow) {
    Billee::SubsystemManagerTester tester;
    tester.testS6_partialDrivetrainWriteFailureDrivesAllLow();
}

TEST(SubsystemManager, S7_OffAlwaysAccepted) {
    Billee::SubsystemManagerTester tester;
    tester.testS7_offAlwaysAccepted();
}

TEST(SubsystemManager, S8_NoAuxInterface) {
    Billee::SubsystemManagerTester tester;
    tester.testS8_noAuxInterface();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
