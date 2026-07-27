module Billee {

    struct PowerReading {
        voltage: F32 @< Voltage reading in volts
        current: F32 @< Current reading in amps
        power: F32 @< Power reading in watts
        sourceId: U8 @< I2C address of the sensor this reading came from
        timestamp: U32 @< Timestamp of reading
    }

}
