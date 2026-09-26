// ======================================================================
// \title  SubsystemManagerTester.hpp
// \brief  hpp file for SubsystemManager component test harness implementation class
// ======================================================================

#ifndef Billee_SubsystemManagerTester_HPP
#define Billee_SubsystemManagerTester_HPP

#include "Components/SubsystemManager/SubsystemManager.hpp"
#include "Components/SubsystemManager/SubsystemManagerGTestBase.hpp"

namespace Billee {

class SubsystemManagerTester final : public SubsystemManagerGTestBase {
  public:
    static const FwSizeType MAX_HISTORY_SIZE = 64;
    static const FwEnumStoreType TEST_INSTANCE_ID = 0;
    static const FwSizeType TEST_INSTANCE_QUEUE_DEPTH = 16;

    //! Simulated pins: 0-5 = DRIVE1-6, 6 = ARM, 7 = SCIENCE
    enum Pin : U8 { PIN_DRIVE1 = 0, PIN_ARM = 6, PIN_SCIENCE = 7, NUM_PINS = 8 };

    SubsystemManagerTester();
    ~SubsystemManagerTester();

    // Tests (ids match the firmware fix plan)
    void testS1_onRejectedWhileEStopEngaged();
    void testS2_onRejectedWhenEStopUnreadable();
    void testS3_onAcceptedWhenReleased();
    void testS4_eStopForcesOffAndReleaseRestoresNothing();
    void testS5_faultInhibitBlocksOn();
    void testS6_partialDrivetrainWriteFailureDrivesAllLow();
    void testS7_offAlwaysAccepted();
    void testS8_noAuxInterface();

  private:
    void connectPorts();
    void initComponents();

    // Helpers
    void setEStopReleased() { this->m_eStopLevel = Fw::Logic::HIGH; this->m_eStopFail = false; }
    void setEStopEngaged() { this->m_eStopLevel = Fw::Logic::LOW; this->m_eStopFail = false; }
    void runOnce();
    void command(Billee::Subsystems subsystem, Fw::On state, U32 cmdSeq);
    bool allDriveLow() const;
    bool allDriveHigh() const;

    // Simulated hardware
    Drv::GpioStatus from_Drive1Set_handler(FwIndexType portNum, const Fw::Logic& state) override;
    Drv::GpioStatus from_Drive2Set_handler(FwIndexType portNum, const Fw::Logic& state) override;
    Drv::GpioStatus from_Drive3Set_handler(FwIndexType portNum, const Fw::Logic& state) override;
    Drv::GpioStatus from_Drive4Set_handler(FwIndexType portNum, const Fw::Logic& state) override;
    Drv::GpioStatus from_Drive5Set_handler(FwIndexType portNum, const Fw::Logic& state) override;
    Drv::GpioStatus from_Drive6Set_handler(FwIndexType portNum, const Fw::Logic& state) override;
    Drv::GpioStatus from_ArmSet_handler(FwIndexType portNum, const Fw::Logic& state) override;
    Drv::GpioStatus from_ScienceSet_handler(FwIndexType portNum, const Fw::Logic& state) override;
    Drv::GpioStatus from_EStopRead_handler(FwIndexType portNum, Fw::Logic& state) override;
    Drv::GpioStatus writePin(U8 pin, const Fw::Logic& state);

    Fw::Logic m_pin[NUM_PINS];
    Fw::Logic m_eStopLevel = Fw::Logic::HIGH;
    bool m_eStopFail = false;
    bool m_failDrive4 = false;

    SubsystemManager component;
};

}  // namespace Billee

#endif
