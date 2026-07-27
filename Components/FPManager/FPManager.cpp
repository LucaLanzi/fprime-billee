// ======================================================================
// \title  FPManager.cpp
// \brief  cpp file for FPManager component implementation class
// ======================================================================

#include "Components/FPManager/FPManager.hpp"

namespace Billee {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

FPManager ::FPManager(const char* const compName) : FPManagerComponentBase(compName) {}

FPManager ::~FPManager() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void FPManager ::powerReadingIn_handler(FwIndexType portNum,
                                        const Billee::Subsystems& subsystem,
                                        const Billee::PowerReading& reading) {
    FaultTracking* tracking = this->trackingFor(subsystem);
    if (tracking == nullptr) {
        return;
    }

    this->loadParamsIfNeeded();

    const F32 voltage = reading.get_voltage();
    if (voltage < this->m_vbusFaultLow) {
        tracking->voltageFault = true;
        tracking->voltageReason = Billee::FaultReason::UNDERVOLTAGE;
    } else if (voltage > this->m_vbusFaultHigh) {
        tracking->voltageFault = true;
        tracking->voltageReason = Billee::FaultReason::OVERVOLTAGE;
    } else {
        tracking->voltageFault = false;
    }

    // INA780B current readings are signed (direction-dependent); compare magnitude
    // against the overcurrent threshold.
    const F32 current = reading.get_current();
    tracking->currentFault = (current > this->m_currentFaultHigh) || (current < -this->m_currentFaultHigh);

    this->evaluate(*tracking, subsystem);
}

void FPManager ::thermalReadingIn_handler(FwIndexType portNum,
                                          const Billee::Subsystems& subsystem,
                                          const Billee::ThermalReading& reading) {
    FaultTracking* tracking = this->trackingFor(subsystem);
    if (tracking == nullptr) {
        return;
    }

    tracking->thermalFault = (reading.get_tempState() == Billee::ThermalStates::FAULT);
    this->evaluate(*tracking, subsystem);
}

// ----------------------------------------------------------------------
// Handler implementation for parameter updates
// ----------------------------------------------------------------------

void FPManager ::parameterUpdated(FwPrmIdType id) {
    switch (id) {
        case PARAMID_VBUS_FAULT_LOW:
            this->m_vbusFaultLow = this->paramGet_VBUS_FAULT_LOW(this->m_paramIsValid);
            this->tlmWrite_VBUS_FAULT_LOW(this->m_vbusFaultLow);
            break;
        case PARAMID_VBUS_FAULT_HIGH:
            this->m_vbusFaultHigh = this->paramGet_VBUS_FAULT_HIGH(this->m_paramIsValid);
            this->tlmWrite_VBUS_FAULT_HIGH(this->m_vbusFaultHigh);
            break;
        case PARAMID_CURRENT_FAULT_HIGH:
            this->m_currentFaultHigh = this->paramGet_CURRENT_FAULT_HIGH(this->m_paramIsValid);
            this->tlmWrite_CURRENT_FAULT_HIGH(this->m_currentFaultHigh);
            break;
        default:
            break;
    }
}

// ----------------------------------------------------------------------
// Helper functions
// ----------------------------------------------------------------------

FPManager::FaultTracking* FPManager ::trackingFor(Billee::Subsystems subsystem) {
    switch (subsystem) {
        case Billee::Subsystems::DRIVETRAIN:
            return &this->m_drivetrain;
        case Billee::Subsystems::ARM:
            return &this->m_arm;
        case Billee::Subsystems::SCIENCE:
            return &this->m_science;
        case Billee::Subsystems::LOGIC:
            return &this->m_logic;
        default:
            // AUX has no power/thermal sensor coverage, so FPManager cannot protect it.
            return nullptr;
    }
}

void FPManager ::evaluate(FaultTracking& tracking, Billee::Subsystems subsystem) {
    const bool faulted = tracking.voltageFault || tracking.currentFault || tracking.thermalFault;
    const bool controllable = (subsystem != Billee::Subsystems::LOGIC);

    if (faulted && !tracking.latched) {
        tracking.latched = true;
        Billee::FaultReason reason = Billee::FaultReason::OVERTEMP;
        if (tracking.voltageFault) {
            reason = tracking.voltageReason;
        } else if (tracking.currentFault) {
            reason = Billee::FaultReason::OVERCURRENT;
        }
        if (controllable) {
            this->emergencyPowerOffOut_out(0, subsystem, Fw::On::OFF);
            this->log_WARNING_HI_SubsystemFaultShutdown(subsystem, reason);
        } else {
            this->log_WARNING_HI_LogicFaultDetected(reason);
        }
    } else if (!faulted && tracking.latched) {
        tracking.latched = false;
        if (controllable) {
            this->log_ACTIVITY_HI_SubsystemFaultCleared(subsystem);
        } else {
            this->log_ACTIVITY_HI_LogicFaultCleared();
        }
    }

    const Billee::FaultState state = tracking.latched ? Billee::FaultState::TRIPPED : Billee::FaultState::NOMINAL;
    switch (subsystem) {
        case Billee::Subsystems::DRIVETRAIN:
            this->tlmWrite_DRIVETRAIN_FAULT_STATE(state);
            break;
        case Billee::Subsystems::ARM:
            this->tlmWrite_ARM_FAULT_STATE(state);
            break;
        case Billee::Subsystems::SCIENCE:
            this->tlmWrite_SCIENCE_FAULT_STATE(state);
            break;
        case Billee::Subsystems::LOGIC:
            this->tlmWrite_LOGIC_FAULT_STATE(state);
            break;
        default:
            break;
    }
}

void FPManager ::loadParamsIfNeeded() {
    if (this->m_paramsLoaded) {
        return;
    }
    this->m_paramsLoaded = true;

    this->m_vbusFaultLow = this->paramGet_VBUS_FAULT_LOW(this->m_paramIsValid);
    this->m_vbusFaultHigh = this->paramGet_VBUS_FAULT_HIGH(this->m_paramIsValid);
    this->m_currentFaultHigh = this->paramGet_CURRENT_FAULT_HIGH(this->m_paramIsValid);
    this->tlmWrite_VBUS_FAULT_LOW(this->m_vbusFaultLow);
    this->tlmWrite_VBUS_FAULT_HIGH(this->m_vbusFaultHigh);
    this->tlmWrite_CURRENT_FAULT_HIGH(this->m_currentFaultHigh);
}

}  // namespace Billee
