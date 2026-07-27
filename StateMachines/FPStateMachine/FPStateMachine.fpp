module Billee {
    state machine FPStateMachine {

        @ Enter NOMINAL on component startup: no fault latched yet
        initial enter NOMINAL

        @ A new power (voltage/current) reading arrived for this subsystem
        signal powerUpdate: Billee.PowerReading

        @ A new thermal reading arrived for this subsystem
        signal thermalUpdate: Billee.ThermalReading

        @ True if the reading (combined with the other domain's last-known status) violates
        @ the configured voltage/current thresholds
        guard isPowerFault: Billee.PowerReading

        @ True if the reading (combined with the other domain's last-known status) is in a
        @ FAULT or FAILURE thermal state
        guard isThermalFault: Billee.ThermalReading

        @ Trip the fault: command the subsystem off (if controllable) and log why
        action doTripFromPower: Billee.PowerReading

        @ Trip the fault: command the subsystem off (if controllable) and log why
        action doTripFromThermal: Billee.ThermalReading

        @ Clear a previously-tripped fault and log the recovery
        action doClear

        @ Decide whether a power reading, while already FAULTED, means we can clear
        choice CHECK_POWER {
            if isPowerFault enter FAULTED \
                else do {doClear} enter NOMINAL
        }

        @ Decide whether a thermal reading, while already FAULTED, means we can clear
        choice CHECK_THERMAL {
            if isThermalFault enter FAULTED \
                else do {doClear} enter NOMINAL
        }

        state NOMINAL {
            on powerUpdate if isPowerFault do {doTripFromPower} enter FAULTED
            on thermalUpdate if isThermalFault do {doTripFromThermal} enter FAULTED
        }

        state FAULTED {
            on powerUpdate enter CHECK_POWER
            on thermalUpdate enter CHECK_THERMAL
        }

    }
}
