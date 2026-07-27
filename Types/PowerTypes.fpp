module Billee {

    @ Identifies which physical INA780B sensor a PowerReading came from
    enum InaSensorId: U8 {
        DRIVE1 = 1 @< Drivetrain motor 1 INA780B (I2C0, 0x40)
        DRIVE2 = 2 @< Drivetrain motor 2 INA780B (I2C0, 0x41)
        DRIVE3 = 3 @< Drivetrain motor 3 INA780B (I2C0, 0x43)
        DRIVE4 = 4 @< Drivetrain motor 4 INA780B (I2C0, 0x44)
        DRIVE5 = 5 @< Drivetrain motor 5 INA780B (I2C0, 0x45)
        DRIVE6 = 6 @< Drivetrain motor 6 INA780B (I2C0, 0x47)
        ARM = 7 @< Arm subsystem INA780B (I2C0, 0x4C)
        SCIENCE = 8 @< Science subsystem INA780B (I2C0, 0x4D)
        LOGIC = 9 @< Logic board INA780B (I2C0, 0x4F)
    }

    struct PowerReading {
        voltage: F32 @< Voltage reading in volts
        current: F32 @< Current reading in amps
        power: F32 @< Power reading in watts
        sourceId: InaSensorId @< Which physical sensor this reading came from
        timestamp: U32 @< Timestamp of reading
    }

    @ Broadcasts a single sensor's power reading, tagged with the subsystem it belongs to.
    @ Used by InaManager to forward readings to FPManager.
    port PowerReadingPort(
        subsystem: Billee.Subsystems @< Subsystem this reading belongs to
        reading: Billee.PowerReading @< The reading itself
    )

}
