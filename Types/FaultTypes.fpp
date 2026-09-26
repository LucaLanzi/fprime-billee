module Billee {

    enum FaultState: U8 {
        NOMINAL = 0 @< No active fault condition
        TRIPPED = 1 @< A fault condition is latched (subsystem inhibited, if controllable) until CLEAR_FAULT
        STALE = 2 @< Per-sensor only: the sensor is not returning valid readings, so it is not being evaluated
    }

    enum FaultReason: U8 {
        UNDERVOLTAGE = 1 @< Bus voltage dropped below the subsystem's configured minimum while commanded ON
        OVERVOLTAGE = 2 @< Bus voltage exceeded the subsystem's configured maximum
        OVERTEMP = 3 @< Thermal sensor reported a FAULT-level temperature state
        OVERCURRENT = 4 @< Current draw exceeded the subsystem's configured cutoff
        SENSOR_FAILURE = 5 @< Reserved: sensor loss is alert-only and never trips (kept for dictionary stability)
    }

    @ Why SubsystemManager refused a power-ON command
    enum PowerOnRejectReason: U8 {
        E_STOP_ENGAGED = 1 @< The E-stop status line reads LOW (stop asserted)
        E_STOP_UNKNOWN = 2 @< The E-stop status line could not be read; treated as engaged
        FAULT_LATCHED = 3 @< FPManager has latched a fault on this subsystem; send fpManager.CLEAR_FAULT first
    }

    @ FPManager -> SubsystemManager: latch (inhibit = true) or release (inhibit = false) a fault
    @ inhibit on a subsystem. Latching forces the subsystem OFF and blocks power-ON commands.
    @ Releasing never turns anything on.
    port FaultInhibit(
        subsystem: Billee.Subsystems @< Subsystem to inhibit/release
        inhibit: bool @< true = force off and block ON; false = allow ON again
    )

    @ SubsystemManager -> FPManager: the actual commanded power state of a subsystem changed
    @ (ground command, E-stop force-off, or fault inhibit). FPManager uses it to gate the
    @ undervoltage check, because the INA780 VBUS pins sit on the switched side and read ~0 V
    @ whenever a channel is off.
    port PowerStateChanged(
        subsystem: Billee.Subsystems @< Subsystem whose state changed
        $state: Fw.On @< New power state
    )

}
