module Billee{

    enum ThermalStates: U8 {
        IDLE = 1 @< System in IDLE mode
        WARN = 2 @< System in WARN mode
        FAULT = 3 @< System in FAULT mode
        NOT_USED = 0 @< Sensor is unavailable or should not be considered
        FAILURE = 4 @< Sensor read failed (e.g. not connected/detected)
    }

    @ Identifies which physical MCP9808 sensor a ThermalReading came from
    enum McpSensorId: U8 {
        LOGIC_TEMP = 1 @< Logic board MCP9808 (I2C1, 0x18)
        DRIVE_TEMP = 2 @< Drivetrain MCP9808 (I2C1, 0x19)
        ARM_SCI_TEMP = 3 @< Arm/science MCP9808 (I2C1, 0x1A)
    }

    struct ThermalReading {
        temperature: F32 @< Temperature in degrees Celsius
        tempState: ThermalStates @< State of the sensor
        sensorId: McpSensorId @< Which physical sensor this reading came from
        location: string size 32 @< Description of sensor location
        timestamp: U32 @< Timestamp of reading
    }

    @ The full configured range of temperature thresholds used to classify a reading into
    @ IDLE/WARN/FAULT, bundled into a single telemetry point so the whole configured range
    @ is visible at a glance
    struct ThermalBounds {
        idleLow: F32 @< IDLE state low threshold
        idleHigh: F32 @< IDLE state high threshold
        warnLow: F32 @< WARNING state low threshold
        warnHigh: F32 @< WARNING state high threshold
        faultLow: F32 @< FAULT state low threshold
        faultHigh: F32 @< FAULT state high threshold
    }

    @ Broadcasts a single sensor's thermal reading, tagged with the subsystem it belongs to.
    @ Used by McpManager to forward readings to FPManager. The shared arm/science sensor
    @ is broadcast twice: once tagged ARM, once tagged SCIENCE.
    port ThermalReadingPort(
        subsystem: Billee.Subsystems @< Subsystem this reading belongs to
        reading: Billee.ThermalReading @< The reading itself
    )

}