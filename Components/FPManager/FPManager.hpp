// ======================================================================
// \title  FPManager.hpp
// \brief  hpp file for FPManager component implementation class
// ======================================================================

#ifndef Billee_FPManager_HPP
#define Billee_FPManager_HPP

#include "Components/FPManager/FPManagerComponentAc.hpp"

namespace Billee {

class FPManager final : public FPManagerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct FPManager object
    FPManager(const char* const compName  //!< The component name
    );

    //! Destroy FPManager object
    ~FPManager();

  private:
    friend class FPManagerTester;  // unit-test access
    //! Monitored subsystems, used as array indices (matches the four state machine instances)
    enum SubIndex : U8 { SUB_DRIVETRAIN = 0, SUB_ARM = 1, SUB_SCIENCE = 2, SUB_LOGIC = 3, NUM_SUBS = 4 };
    static constexpr U8 NUM_INA = 9;  //!< InaSensorId 1..9
    static constexpr U8 NUM_MCP = 3;  //!< McpSensorId 1..3

    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    void powerReadingIn_handler(FwIndexType portNum,
                                const Billee::Subsystems& subsystem,
                                const Billee::PowerReading& reading) override;

    void thermalReadingIn_handler(FwIndexType portNum,
                                  const Billee::Subsystems& subsystem,
                                  const Billee::ThermalReading& reading) override;

    void powerStateIn_handler(FwIndexType portNum,
                              const Billee::Subsystems& subsystem,
                              const Fw::On& state) override;

    //! Queue-overflow hooks (run on the sender's thread): count and report dropped readings
    void powerReadingIn_overflowHook(FwIndexType portNum,
                                     const Billee::Subsystems& subsystem,
                                     const Billee::PowerReading& reading) override;

    void thermalReadingIn_overflowHook(FwIndexType portNum,
                                       const Billee::Subsystems& subsystem,
                                       const Billee::ThermalReading& reading) override;

    void powerStateIn_overflowHook(FwIndexType portNum,
                                   const Billee::Subsystems& subsystem,
                                   const Fw::On& state) override;

    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    void CLEAR_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Billee::Subsystems subsystem) override;

    //! Parameter update notification
    void parameterUpdated(FwPrmIdType id) override;

    // ----------------------------------------------------------------------
    // State machine actions
    // ----------------------------------------------------------------------

    void Billee_FPStateMachine_action_doTrip(SmId smId,
                                             Billee_FPStateMachine::Signal signal,
                                             const Billee::FaultReason& value) override;

    void Billee_FPStateMachine_action_doClear(SmId smId, Billee_FPStateMachine::Signal signal) override;

    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    static bool subIndexFor(Billee::Subsystems subsystem, U8& index);
    static U8 subIndexFor(SmId smId);
    static Billee::Subsystems subsystemFor(U8 subIndex);
    static bool isControllable(U8 subIndex);

    void applyPowerState(U8 subIndex, bool on);
    void applyPendingPowerStates();
    bool isFaulted(U8 subIndex);
    void sendFault(U8 subIndex, Billee::FaultReason reason);
    void sendClear(U8 subIndex);

    F64 nowSeconds();
    void loadParamsIfNeeded();
    void loadParams();
    void publishThresholdTelemetry();
    void writeFaultStateTelemetry(U8 subIndex, Billee::FaultState state);
    void writeSensorStateTelemetry(Billee::InaSensorId sensorId, Billee::FaultState state);
    void resetSubsystemCounters(U8 subIndex);

    //! Per-INA780 debounce counters (saturating)
    struct SensorEval {
        U8 oc = 0;
        U8 ov = 0;
        U8 uv = 0;
        U8 subIndex = SUB_LOGIC;
    };

    // ----------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------

    SensorEval m_sensor[NUM_INA];
    bool m_commandedOn[NUM_SUBS] = {false, false, false, true};  //!< LOGIC is always on
    F64 m_onSince[NUM_SUBS] = {0.0, 0.0, 0.0, 0.0};
    U8 m_thermalCount[NUM_SUBS] = {0, 0, 0, 0};
    bool m_thermalFault[NUM_SUBS] = {false, false, false, false};  //!< Latest thermal state is FAULT
    bool m_mcpLost[NUM_MCP] = {false, false, false};

    // Parameters (cached)
    bool m_paramsLoaded = false;
    Billee::PowerBounds m_bounds[NUM_SUBS];
    F32 m_uvSettleS = 1.0f;
    U8 m_debounceOc = 3;
    U8 m_debounceOv = 3;
    U8 m_debounceUv = 5;
    U8 m_debounceThermal = 2;

    //! Power-state changes parked by powerStateIn_overflowHook (sender thread):
    //! 0 = none, 1 = OFF, 2 = ON. Single-byte writes; applied on FPManager's thread.
    volatile U8 m_pendingPowerState[NUM_SUBS] = {0, 0, 0, 0};

    // Overflow accounting (written from sender threads)
    volatile U32 m_droppedReadings = 0;
    volatile bool m_dropReported = false;
};

}  // namespace Billee

#endif
