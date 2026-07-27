module Billee {

    @ Fault Protection Manager. Layers software fault protection on top of the
    @ subsystem power (InaManager) and thermal (McpManager) sensors: 6S LiPo
    @ bus-voltage protection (in addition to the existing hardware failsafe) and
    @ thermal-fault protection. On a fault, commands SubsystemManager to power off
    @ the affected subsystem and logs the reason. Passive: it reacts to readings
    @ pushed in by InaManager/McpManager rather than polling on its own schedule.
    passive component FPManager {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Power (voltage/current) reading from InaManager, called once per sensor per poll cycle
        guarded input port powerReadingIn: Billee.PowerReadingPort

        @ Thermal reading from McpManager, called once per sensor per poll cycle (the shared
        @ arm/science sensor arrives twice: once tagged ARM, once tagged SCIENCE)
        guarded input port thermalReadingIn: Billee.ThermalReadingPort

        @ Commands SubsystemManager to power off a subsystem that has tripped a fault
        output port emergencyPowerOffOut: Billee.SetPowerState

        # ----------------------------------------------------------------------
        # Parameters: 6S LiPo bus-voltage protection thresholds
        # ----------------------------------------------------------------------

        @ Undervoltage fault threshold in volts (3.2 V/cell nominal cutoff for a 6S pack)
        param VBUS_FAULT_LOW: F32 \
            default 19.2 \
            id 0x00 \
            set opcode 0x01 \
            save opcode 0x02

        @ Overvoltage fault threshold in volts (4.25 V/cell for a 6S pack)
        param VBUS_FAULT_HIGH: F32 \
            default 25.5 \
            id 0x01 \
            set opcode 0x03 \
            save opcode 0x04

        @ Overcurrent fault threshold in amps, applied uniformly to every monitored subsystem
        param CURRENT_FAULT_HIGH: F32 \
            default 15.0 \
            id 0x02 \
            set opcode 0x05 \
            save opcode 0x06

        @ Telemetry mirror of the undervoltage threshold
        telemetry VBUS_FAULT_LOW: F32 id 0x10

        @ Telemetry mirror of the overvoltage threshold
        telemetry VBUS_FAULT_HIGH: F32 id 0x11

        @ Telemetry mirror of the overcurrent threshold
        telemetry CURRENT_FAULT_HIGH: F32 id 0x12

        # ----------------------------------------------------------------------
        # Telemetry: latched fault state per monitored subsystem
        # ----------------------------------------------------------------------

        @ Fault-protection state of the drivetrain subsystem
        telemetry DRIVETRAIN_FAULT_STATE: Billee.FaultState id 0

        @ Fault-protection state of the arm subsystem
        telemetry ARM_FAULT_STATE: Billee.FaultState id 1

        @ Fault-protection state of the science subsystem
        telemetry SCIENCE_FAULT_STATE: Billee.FaultState id 2

        @ Fault-protection state of the logic board (monitoring only, no power control)
        telemetry LOGIC_FAULT_STATE: Billee.FaultState id 3

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ A subsystem tripped a fault and was commanded off
        event SubsystemFaultShutdown(
            subsystemName: Billee.Subsystems
            reason: Billee.FaultReason
        ) \
            severity warning high \
            id 0 \
            format "Subsystem {} commanded OFF by FPManager due to {}"

        @ A subsystem's fault condition cleared (readings back within nominal range)
        event SubsystemFaultCleared(
            subsystemName: Billee.Subsystems
        ) \
            severity activity high \
            id 1 \
            format "Subsystem {} fault-protection condition cleared"

        @ The logic board (flight computer) tripped a fault. FPManager cannot power-cycle
        @ it (it is the computer FPManager itself runs on); this is an alert-only event.
        event LogicFaultDetected(
            reason: Billee.FaultReason
        ) \
            severity warning high \
            id 2 \
            format "Logic board fault detected: {} (cannot be power-cycled by FPManager)"

        @ The logic board fault condition cleared
        event LogicFaultCleared() \
            severity activity high \
            id 3 \
            format "Logic board fault-protection condition cleared"

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Port to return the value of a parameter
        param get port prmGetOut

        @ Port to set the value of a parameter
        param set port prmSetOut

    }

}
