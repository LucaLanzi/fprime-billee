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
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for powerReadingIn
    void powerReadingIn_handler(FwIndexType portNum,       //!< The port number
                                const Billee::Subsystems& subsystem,
                                const Billee::PowerReading& reading) override;

    //! Handler implementation for thermalReadingIn
    void thermalReadingIn_handler(FwIndexType portNum,     //!< The port number
                                  const Billee::Subsystems& subsystem,
                                  const Billee::ThermalReading& reading) override;

    // ----------------------------------------------------------------------
    // Handler implementation for parameter updates
    // ----------------------------------------------------------------------
    void parameterUpdated(FwPrmIdType id) override;

  private:
    // ----------------------------------------------------------------------
    // Implementations for internal state machine guards
    // ----------------------------------------------------------------------

    //! True if the incoming power reading, combined with this subsystem's last-known thermal
    //! status, means the subsystem should be (or remain) faulted
    bool Billee_FPStateMachine_guard_isPowerFault(SmId smId,
                                                  Billee_FPStateMachine::Signal signal,
                                                  const Billee::PowerReading& data) const override;

    //! True if the incoming thermal reading, combined with this subsystem's last-known power
    //! status, means the subsystem should be (or remain) faulted
    bool Billee_FPStateMachine_guard_isThermalFault(SmId smId,
                                                    Billee_FPStateMachine::Signal signal,
                                                    const Billee::ThermalReading& data) const override;

    // ----------------------------------------------------------------------
    // Implementations for internal state machine actions
    // ----------------------------------------------------------------------

    //! Trip a fault caused by a voltage/current violation: command the subsystem off (if
    //! controllable) and log why
    void Billee_FPStateMachine_action_doTripFromPower(SmId smId,
                                                      Billee_FPStateMachine::Signal signal,
                                                      const Billee::PowerReading& data) override;

    //! Trip a fault caused by a thermal violation: command the subsystem off (if controllable)
    //! and log why
    void Billee_FPStateMachine_action_doTripFromThermal(SmId smId,
                                                        Billee_FPStateMachine::Signal signal,
                                                        const Billee::ThermalReading& data) override;

    //! Clear a previously-tripped fault and log the recovery
    void Billee_FPStateMachine_action_doClear(SmId smId, Billee_FPStateMachine::Signal signal) override;

  private:
    // ----------------------------------------------------------------------
    // Per-subsystem fault-domain cache
    // ----------------------------------------------------------------------

    //! Each state machine instance only receives ONE domain's data per signal; this cache
    //! remembers the OTHER domain's last-known status so a guard can still correctly combine
    //! both when deciding the overall fault condition.
    struct FaultCache {
        bool powerFaulted = false;
        bool thermalFaulted = false;
    };

    FaultCache m_drivetrainCache;
    FaultCache m_armCache;
    FaultCache m_scienceCache;
    FaultCache m_logicCache;  //!< Monitoring only: LOGIC has no power control (it runs FPManager)

    FaultCache& cacheFor(SmId smId);
    const FaultCache& cacheFor(SmId smId) const;

    static Billee::Subsystems subsystemFor(SmId smId);
    static bool isControllable(SmId smId);
    void writeFaultStateTelemetry(SmId smId, Billee::FaultState state);

    //! Evaluates a single reading against the cached voltage/current thresholds (independent
    //! of, and finer-grained than, the latched per-subsystem state machines) and writes the
    //! result to that physical sensor's own *_POWER_STATE telemetry channel
    void writePowerSensorStateTelemetry(const Billee::PowerReading& reading);

    //! True if the reading's voltage/current violates the cached thresholds. Shared by the
    //! isPowerFault guard and writePowerSensorStateTelemetry so the fault formula lives in
    //! exactly one place.
    bool isPowerReadingOutOfBounds(const Billee::PowerReading& reading) const;

    // ----------------------------------------------------------------------
    // 6S LiPo bus-voltage + overcurrent protection thresholds (cached copies of the params)
    // ----------------------------------------------------------------------

    bool m_paramsLoaded = false;
    F32 m_vbusFaultLow = 0.0f;
    F32 m_vbusFaultHigh = 0.0f;
    F32 m_currentFaultHigh = 0.0f;
    Fw::ParamValid m_paramIsValid = Fw::ParamValid::VALID;

    //! Lazily loads params on first use (guards are const and cannot call the non-const
    //! paramGet, so the handlers load/cache them here, before signaling the state machine)
    void loadParamsIfNeeded();

    //! Publishes the current threshold values to telemetry
    void publishThresholdTelemetry();
};

}  // namespace Billee

#endif
