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

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    // Member functions and types for managing subsystem power states
    Fw::On m_drivetrainState = Fw::On::OFF;
    Fw::On m_armState = Fw::On::OFF;
    Fw::On m_scienceState = Fw::On::OFF;
    Fw::On m_auxState = Fw::On::OFF;
    static Fw::Logic toLogic(Fw::On state);
    static bool gpioWriteSucceeded(Drv::GpioStatus status);

    bool setDrivetrainGpios(Fw::Logic state);
    //! Handler implementation for run
    //!
    //! Input port invoked by the rate group
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;
  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------
    //! Handler implementation for command SET_DRIVETRAIN_POWER_STATE
    //!
    //! Set the drivetrain subsystem power state
    void SET_DRIVETRAIN_POWER_STATE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                               U32 cmdSeq,           //!< The command sequence number
                                               Fw::On driveState     //!< Requested power state
                                               ) override;
    //! Handler implementation for command SET_ARM_POWER_STATE
    //!
    //! Set the arm subsystem power state
    void SET_ARM_POWER_STATE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                        U32 cmdSeq,           //!< The command sequence number
                                        Fw::On armState       //!< Requested power state
                                        ) override;
    //! Handler implementation for command SET_AUX_POWER_STATE
    //!
    //! Set the auxiliary subsystem power state
    void SET_AUX_POWER_STATE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                        U32 cmdSeq,           //!< The command sequence number
                                        Fw::On auxState       //!< Requested power state
                                        ) override;
    //! Handler implementation for command SET_SCIENCE_POWER_STATE
    //!
    //! Set the science subsystem power state
    void SET_SCIENCE_POWER_STATE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                            U32 cmdSeq,           //!< The command sequence number
                                            Fw::On scienceState   //!< Requested power state
                                            ) override;
};

}  // namespace Billee
#endif
