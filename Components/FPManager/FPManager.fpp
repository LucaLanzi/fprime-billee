module Billee {

    @ Fault Protection Manager. Layers software fault protection on top of the
    @ subsystem power (InaManager) and thermal (McpManager) sensors: 6S LiPo
    @ bus-voltage protection (in addition to the existing hardware failsafe) and
    @ thermal-fault protection. On a fault, commands SubsystemManager to power off
    @ the affected subsystem and logs the reason. Active (state machines require an
    @ owning thread): reacts to readings pushed in by InaManager/McpManager rather
    @ than polling on its own schedule.
    active component FPManager {

        # ----------------------------------------------------------------------
        # State machine instances: one per monitored subsystem, tracking the
        # combined power+thermal fault status independently for each
        # ----------------------------------------------------------------------

        @ Fault-protection state machine for the drivetrain subsystem
        state machine instance fp_drivetrainSM: FPStateMachine

        @ Fault-protection state machine for the arm subsystem
        state machine instance fp_armSM: FPStateMachine

        @ Fault-protection state machine for the science subsystem
        state machine instance fp_scienceSM: FPStateMachine

        @ Fault-protection state machine for the logic board (monitoring only)
        state machine instance fp_logicSM: FPStateMachine

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Power (voltage/current) reading from InaManager, called once per sensor per poll cycle.
        @ InaManager fires 9 of these back-to-back in a single run_handler call, so this must
        @ tolerate a burst larger than one message; `drop` (instead of the default assert-on-
        @ overflow) means a burst that outruns the queue just loses a stale reading rather than
        @ crashing the whole rate group -- unacceptable for a fault-protection component.
        async input port powerReadingIn: Billee.PowerReadingPort drop

        @ Thermal reading from McpManager, called once per sensor per poll cycle (the shared
        @ arm/science sensor arrives twice: once tagged ARM, once tagged SCIENCE). See
        @ powerReadingIn above for why this uses `drop`.
        async input port thermalReadingIn: Billee.ThermalReadingPort drop

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

        # The full configured range of voltage/current fault thresholds, bundled into a single
        # telemetry point per subsystem so the bounds applied to each subsystem's fault
        # protection are visible at a glance. All four currently share the same underlying
        # VBUS_FAULT_LOW/HIGH and CURRENT_FAULT_HIGH params (there is one global threshold set,
        # not one per subsystem), but are exposed per subsystem for clarity on the ground.

        @ Voltage/current fault-protection bounds applied to the drivetrain subsystem (all 6
        @ drive motors share this one set of thresholds)
        telemetry DRIVETRAIN_POWER_BOUNDS: Billee.PowerBounds id 0x10

        @ Voltage/current fault-protection bounds applied to the arm subsystem
        telemetry ARM_POWER_BOUNDS: Billee.PowerBounds id 0x11

        @ Voltage/current fault-protection bounds applied to the science subsystem
        telemetry SCIENCE_POWER_BOUNDS: Billee.PowerBounds id 0x12

        @ Voltage/current fault-protection bounds applied to the logic board
        telemetry LOGIC_POWER_BOUNDS: Billee.PowerBounds id 0x13

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
        # Telemetry: per-physical-sensor voltage/current fault state, evaluated against
        # VBUS_FAULT_LOW/HIGH and CURRENT_FAULT_HIGH on every individual reading (finer-grained
        # than the per-subsystem *_FAULT_STATE channels above, which combine power+thermal and
        # merge all 6 drivetrain sensors into one state)
        # ----------------------------------------------------------------------

        @ Voltage/current fault state of the drivetrain motor 1 INA780B
        telemetry DRIVE1_POWER_STATE: Billee.FaultState id 4

        @ Voltage/current fault state of the drivetrain motor 2 INA780B
        telemetry DRIVE2_POWER_STATE: Billee.FaultState id 5

        @ Voltage/current fault state of the drivetrain motor 3 INA780B
        telemetry DRIVE3_POWER_STATE: Billee.FaultState id 6

        @ Voltage/current fault state of the drivetrain motor 4 INA780B
        telemetry DRIVE4_POWER_STATE: Billee.FaultState id 7

        @ Voltage/current fault state of the drivetrain motor 5 INA780B
        telemetry DRIVE5_POWER_STATE: Billee.FaultState id 8

        @ Voltage/current fault state of the drivetrain motor 6 INA780B
        telemetry DRIVE6_POWER_STATE: Billee.FaultState id 9

        @ Voltage/current fault state of the arm subsystem INA780B
        telemetry ARM_POWER_STATE: Billee.FaultState id 10

        @ Voltage/current fault state of the science subsystem INA780B
        telemetry SCIENCE_POWER_STATE: Billee.FaultState id 11

        @ Voltage/current fault state of the logic board INA780B
        telemetry LOGIC_POWER_STATE: Billee.FaultState id 12

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
