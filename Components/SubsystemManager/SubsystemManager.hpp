// ======================================================================
// \title  SubsystemManager.hpp
// \author luquito
// \brief  hpp file for SubsystemManager component implementation class
// ======================================================================

#ifndef Billee_SubsystemManager_HPP
#define Billee_SubsystemManager_HPP
#include "Components/SubsystemManager/SubsystemManagerComponentAc.hpp"

namespace Billee {

class SubsystemManager final : public SubsystemManagerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct SubsystemManager object
    SubsystemManager(const char* const compName  //!< The component name
    );
    //! Destroy SubsystemManager object
    ~SubsystemManager();

    //! E-STOP line state as seen by this component
    enum class EStopState : U8 {
        UNKNOWN,   //!< Not read yet, or the last read failed (treated as engaged)
        ENGAGED,   //!< Line LOW: stop asserted
        RELEASED,  //!< Line HIGH: run allowed
    };

  private:
    friend class SubsystemManagerTester;  // unit-test access
    //! Controllable subsystems, used as array indices
    enum Index : U8 { IDX_DRIVETRAIN = 0, IDX_ARM = 1, IDX_SCIENCE = 2, NUM_CONTROLLED = 3 };

    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run: polls the E-STOP line and publishes state
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

    //! Handler implementation for faultInhibitIn
    void faultInhibitIn_handler(FwIndexType portNum,                 //!< The port number
                                const Billee::Subsystems& subsystem,  //!< Subsystem to inhibit/release
                                bool inhibit                          //!< true = inhibit, false = release
                                ) override;

    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    void SET_DRIVETRAIN_POWER_STATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Fw::On driveState) override;
    void SET_ARM_POWER_STATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Fw::On armState) override;
    void SET_SCIENCE_POWER_STATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Fw::On scienceState) override;

    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    //! Shared command path for all SET_*_POWER_STATE commands
    void handlePowerCommand(FwOpcodeType opCode, U32 cmdSeq, Billee::Subsystems subsystem, Fw::On requested);

    //! Read the E-STOP line now. A failed read returns UNKNOWN.
    EStopState readEStop();

    //! Single choke point for every power-state change. Writes the GPIO(s), updates state,
    //! emits event/telemetry and notifies FPManager on change. On a GPIO write failure the
    //! subsystem's enables are all driven LOW and the state is OFF. Returns false on failure.
    bool applyState(Billee::Subsystems subsystem, Fw::On state);

    //! Write the GPIO(s) of one subsystem
    bool writeGpios(Billee::Subsystems subsystem, Fw::Logic level);

    //! Force every subsystem that is ON to OFF. Returns true if anything was ON.
    bool forceAllOff();

    void writeStateTelemetry(Billee::Subsystems subsystem);

    static bool toIndex(Billee::Subsystems subsystem, U8& index);
    static Billee::Subsystems fromIndex(U8 index);
    static Fw::Logic toLogic(Fw::On state);
    static bool gpioOpSucceeded(Drv::GpioStatus status);

    // ----------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------

    Fw::On m_state[NUM_CONTROLLED] = {Fw::On::OFF, Fw::On::OFF, Fw::On::OFF};
    bool m_inhibited[NUM_CONTROLLED] = {false, false, false};
    EStopState m_eStop = EStopState::UNKNOWN;
    bool m_eStopReadFailed = false;
};

}  // namespace Billee
#endif
