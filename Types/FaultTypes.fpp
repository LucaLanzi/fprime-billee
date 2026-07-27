module Billee {

    enum FaultState: U8 {
        NOMINAL = 0 @< No active fault condition
        TRIPPED = 1 @< A fault condition is active (subsystem has been commanded off, if controllable)
    }

    enum FaultReason: U8 {
        UNDERVOLTAGE = 1 @< Bus voltage dropped below the configured 6S LiPo cutoff
        OVERVOLTAGE = 2 @< Bus voltage exceeded the configured 6S LiPo cutoff
        OVERTEMP = 3 @< Thermal sensor reported a FAULT-level temperature state
        OVERCURRENT = 4 @< Current draw exceeded the configured cutoff
        SENSOR_FAILURE = 5 @< Thermal sensor reported a FAILURE state (not connected/detected)
    }

    @ Commands the receiving component (SubsystemManager) to set a subsystem's power state.
    @ Used by FPManager to autonomously power off a subsystem that tripped a fault,
    @ bypassing the normal ground-command path.
    port SetPowerState(
        subsystem: Billee.Subsystems @< Subsystem to control
        $state: Fw.On @< Requested power state
    )

}
