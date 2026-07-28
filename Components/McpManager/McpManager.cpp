// ======================================================================
// \title  McpManager.cpp
// \brief  cpp file for McpManager component implementation class
// ======================================================================

#include "Components/McpManager/McpManager.hpp"

namespace Billee {

namespace {
const char* locationForIndex(U8 index) {
    switch (index) {
        case 0:
            return "Logic";
        case 1:
            return "Drivetrain";
        case 2:
            return "ArmScience";
        default:
            return "Unknown";
    }
}

Billee::McpSensorId sensorIdForIndex(U8 index) {
    switch (index) {
        case 0:
            return Billee::McpSensorId::LOGIC_TEMP;
        case 1:
            return Billee::McpSensorId::DRIVE_TEMP;
        case 2:
        default:
            return Billee::McpSensorId::ARM_SCI_TEMP;
    }
}

// Converts the raw 2-byte MCP9808 ambient temperature register into degrees Celsius.
F32 convertRawTemp(const U8* rawData) {
    U8 upperByte = rawData[0] & 0x1F;  // Clear flag bits, keep only temperature data
    const U8 lowerByte = rawData[1];

    if ((upperByte & 0x10) == 0x10) {  // Sign bit set: negative temperature
        upperByte &= 0x0F;             // Clear sign bit
        return 256.0f - ((upperByte * 16.0f) + (lowerByte / 16.0f));
    }
    return (upperByte * 16.0f) + (lowerByte / 16.0f);
}
}  // namespace

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

McpManager ::McpManager(const char* const compName)
    : McpManagerComponentBase(compName), m_justBooted(true), m_successfulRead(true) {
    deviceAddrs[0] = LOGIC_TEMP_ADDR;
    deviceAddrs[1] = DRIVE_TEMP_ADDR;
    deviceAddrs[2] = ARM_SCI_TEMP_ADDR;
}

McpManager ::~McpManager() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void McpManager ::run_handler(FwIndexType portNum, U32 context) {
    this->mcp_thermalStateMachine_sendSignal_tick();
}

// ----------------------------------------------------------------------
// Implementations for internal state machine actions
// ----------------------------------------------------------------------

void McpManager ::Billee_ThermalStateMachine_action_doRead(SmId smId, Billee_ThermalStateMachine::Signal signal) {
    if (this->m_justBooted) {
        this->m_justBooted = false;
        this->m_startTime = this->getTime().getSeconds();  // Record boot time to track uptime in telemetry

        this->IDLE_LOW_THR = this->paramGet_MCP_IDLE_LOW(m_paramIsValid);
        this->IDLE_HIGH_THR = this->paramGet_MCP_IDLE_HIGH(m_paramIsValid);
        this->WARN_LOW_THR = this->paramGet_MCP_WARN_LOW(m_paramIsValid);
        this->WARN_HIGH_THR = this->paramGet_MCP_WARN_HIGH(m_paramIsValid);
        this->FAULT_LOW_THR = this->paramGet_MCP_FAULT_LOW(m_paramIsValid);
        this->FAULT_HIGH_THR = this->paramGet_MCP_FAULT_HIGH(m_paramIsValid);
        this->publishBoundsTelemetry();

        // Skip straight to a read on the very next tick rather than waiting a full cycle idle.
        this->mcp_thermalStateMachine_sendSignal_success();
        return;
    }

    for (U8 i = 0; i < 3; i++) {
        F32 tempCelsius = 0.0f;
        this->m_sensorOk[i] = this->readTemp(this->deviceAddrs[i], tempCelsius);
        if (this->m_sensorOk[i]) {
            this->m_thermalReadings[i].set_temperature(tempCelsius);
        } else {
            this->m_thermalReadings[i].set_temperature(0.0f);
            this->m_successfulRead = false;
        }

        this->m_thermalReadings[i].set_sensorId(sensorIdForIndex(i));
        this->m_thermalReadings[i].set_timestamp(this->getTime().getSeconds() - this->m_startTime);
        this->m_thermalReadings[i].set_location(Fw::String(locationForIndex(i)));
    }

    // Either way, telemetry is always published (see doEvaluate/doReadFail/publishReadings):
    // a failed sensor just reports tempState FAILURE instead of being silently skipped.
    if (this->m_successfulRead) {
        this->mcp_thermalStateMachine_sendSignal_success();
    } else {
        this->mcp_thermalStateMachine_sendSignal_fail();
    }
}

void McpManager ::Billee_ThermalStateMachine_action_doEvaluate(SmId smId,
                                                                Billee_ThermalStateMachine::Signal signal) {
    if (this->m_wasFailed) {
        this->m_wasFailed = false;
        this->log_ACTIVITY_HI_McpReadRecovered();
    }

    for (U8 i = 0; i < 3; i++) {
        const Billee::ThermalStates tempState = this->determineTempState(this->m_thermalReadings[i].get_temperature());
        this->m_thermalReadings[i].set_tempState(tempState);
    }
    this->publishReadings();

    this->mcp_thermalStateMachine_sendSignal_success();  // Loop back to read again on the next tick
}

void McpManager ::Billee_ThermalStateMachine_action_doReadFail(SmId smId,
                                                                Billee_ThermalStateMachine::Signal signal) {
    if (!this->m_wasFailed) {
        this->m_wasFailed = true;
        this->log_WARNING_HI_McpReadFailure();
    }

    for (U8 i = 0; i < 3; i++) {
        Billee::ThermalStates tempState = Billee::ThermalStates::FAILURE;
        if (this->m_sensorOk[i]) {
            tempState = this->determineTempState(this->m_thermalReadings[i].get_temperature());
        }
        this->m_thermalReadings[i].set_tempState(tempState);
    }
    this->publishReadings();

    this->m_successfulRead = true;  // Reset so the next tick tries reading again
    this->mcp_thermalStateMachine_sendSignal_success();
}

// ----------------------------------------------------------------------
// Handler implementations for parameters update
// ----------------------------------------------------------------------

void McpManager ::parameterUpdated(FwPrmIdType id) {
    switch (id) {
        case PARAMID_MCP_IDLE_LOW:
            this->IDLE_LOW_THR = this->paramGet_MCP_IDLE_LOW(m_paramIsValid);
            break;
        case PARAMID_MCP_IDLE_HIGH:
            this->IDLE_HIGH_THR = this->paramGet_MCP_IDLE_HIGH(m_paramIsValid);
            break;
        case PARAMID_MCP_WARN_LOW:
            this->WARN_LOW_THR = this->paramGet_MCP_WARN_LOW(m_paramIsValid);
            break;
        case PARAMID_MCP_WARN_HIGH:
            this->WARN_HIGH_THR = this->paramGet_MCP_WARN_HIGH(m_paramIsValid);
            break;
        case PARAMID_MCP_FAULT_LOW:
            this->FAULT_LOW_THR = this->paramGet_MCP_FAULT_LOW(m_paramIsValid);
            break;
        case PARAMID_MCP_FAULT_HIGH:
            this->FAULT_HIGH_THR = this->paramGet_MCP_FAULT_HIGH(m_paramIsValid);
            break;
        default:
            break;
    }
    this->publishBoundsTelemetry();
}

// ----------------------------------------------------------------------
// Helper functions
// ----------------------------------------------------------------------

bool McpManager ::readTemp(U8 deviceAddr, F32& temperature) {
    U8 regAddr = TEMP_REG_ADDR;
    U8 rawData[2];  // MCP9808 ambient temperature register is 2 bytes
    Fw::Buffer writeBuffer(&regAddr, 1);
    Fw::Buffer readBuffer(rawData, 2);

    const Drv::I2cStatus status = this->mcpWriteRead_out(0, deviceAddr, writeBuffer, readBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        temperature = 0.0f;
        return false;
    }

    temperature = convertRawTemp(rawData);
    return true;
}

void McpManager ::publishReadings() {
    this->tlmWrite_LOGIC_TEMP(this->m_thermalReadings[0]);
    this->thermalReadingOut_out(0, Billee::Subsystems::LOGIC, this->m_thermalReadings[0]);

    this->tlmWrite_DRIVE_TEMP(this->m_thermalReadings[1]);
    this->thermalReadingOut_out(0, Billee::Subsystems::DRIVETRAIN, this->m_thermalReadings[1]);

    this->tlmWrite_ARM_SCI_TEMP(this->m_thermalReadings[2]);
    this->thermalReadingOut_out(0, Billee::Subsystems::ARM, this->m_thermalReadings[2]);
    this->thermalReadingOut_out(0, Billee::Subsystems::SCIENCE, this->m_thermalReadings[2]);

    // Republished every cycle (not just once at boot) so a GDS client connecting after boot
    // still sees the current bounds without waiting for a param change.
    this->publishBoundsTelemetry();
}

void McpManager ::publishBoundsTelemetry() {
    Billee::ThermalBounds bounds;
    bounds.set_idleLow(this->IDLE_LOW_THR);
    bounds.set_idleHigh(this->IDLE_HIGH_THR);
    bounds.set_warnLow(this->WARN_LOW_THR);
    bounds.set_warnHigh(this->WARN_HIGH_THR);
    bounds.set_faultLow(this->FAULT_LOW_THR);
    bounds.set_faultHigh(this->FAULT_HIGH_THR);
    this->tlmWrite_MCP_TEMP_BOUNDS(bounds);
}

Billee::ThermalStates McpManager ::determineTempState(F32 tempCelsius) {
    if (this->IDLE_LOW_THR <= tempCelsius && tempCelsius <= this->IDLE_HIGH_THR) {
        return Billee::ThermalStates::IDLE;
    }
    if ((this->WARN_LOW_THR <= tempCelsius && tempCelsius < this->IDLE_LOW_THR) ||
        (this->IDLE_HIGH_THR < tempCelsius && tempCelsius <= this->WARN_HIGH_THR)) {
        return Billee::ThermalStates::WARN;
    }
    return Billee::ThermalStates::FAULT;
}

}  // namespace Billee
