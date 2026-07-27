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
    // Per-subsystem fault-tracking state
    // ----------------------------------------------------------------------

    //! Latest voltage/thermal fault inputs and the latched overall state for one subsystem.
    //! Voltage and thermal readings arrive independently and asynchronously; the latched
    //! state is only re-evaluated (and acted on) when either input changes.
    struct FaultTracking {
        bool voltageFault = false;
        bool currentFault = false;
        bool thermalFault = false;
        bool latched = false;
        Billee::FaultReason voltageReason = Billee::FaultReason::UNDERVOLTAGE;
    };

    FaultTracking m_drivetrain;
    FaultTracking m_arm;
    FaultTracking m_science;
    FaultTracking m_logic;  //!< Monitoring only: LOGIC has no power control (it runs FPManager)

    //! Returns the tracking state for a subsystem, or nullptr if FPManager doesn't monitor it
    //! (e.g. AUX, which has no power or thermal sensor coverage).
    FaultTracking* trackingFor(Billee::Subsystems subsystem);

    //! Re-checks the combined (voltage OR thermal) fault condition for a subsystem against its
    //! latched state, acting (and logging) only on a state transition (edge-triggered), then
    //! publishes the resulting telemetry.
    void evaluate(FaultTracking& tracking, Billee::Subsystems subsystem);

    // ----------------------------------------------------------------------
    // 6S LiPo bus-voltage protection thresholds (cached copies of the params)
    // ----------------------------------------------------------------------

    bool m_paramsLoaded = false;
    F32 m_vbusFaultLow = 0.0f;
    F32 m_vbusFaultHigh = 0.0f;
    F32 m_currentFaultHigh = 0.0f;
    Fw::ParamValid m_paramIsValid = Fw::ParamValid::VALID;

    //! Lazily loads params on first use (PrmDb is only guaranteed loaded once the topology
    //! has finished starting up, which has already happened by the time any sensor reading
    //! reaches this passive component).
    void loadParamsIfNeeded();
};

}  // namespace Billee

#endif
