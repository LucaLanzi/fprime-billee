module Billee{

    enum ThermalStates: U8 {
        IDLE = 1 @< System in IDLE mode
        WARN = 2 @< System in WARN mode
        FAULT = 3 @< System in FAULT mode
        NOT_USED = 0 @< Sensor is unavailable or should not be considered
    }

    struct ThermalReading {
        temperature: F32 @< Temperature in degrees Celsius
        tempState: ThermalStates @< State of the sensor
        sensorId: U8 @< ID of the thermal sensor
        location: string size 32 @< Description of sensor location
        timestamp: U32 @< Timestamp of reading
    }

    @ Broadcasts a single sensor's thermal reading, tagged with the subsystem it belongs to.
    @ Used by McpManager to forward readings to FPManager. The shared arm/science sensor
    @ is broadcast twice: once tagged ARM, once tagged SCIENCE.
    port ThermalReadingPort(
        subsystem: Billee.Subsystems @< Subsystem this reading belongs to
        reading: Billee.ThermalReading @< The reading itself
    )

}