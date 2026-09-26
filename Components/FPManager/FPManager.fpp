module Billee {

    @ Fault Protection Manager. Layers software fault protection on top of the subsystem power
    @ (InaManager) and thermal (McpManager) sensors. Each INA780 reading is evaluated per sensor
    @ against its subsystem's bounds with consecutive-sample debounce. A debounced fault latches
    @ the subsystem's FPStateMachine: controllable subsystems are forced OFF and ON is blocked in
    @ SubsystemManager until the ground sends CLEAR_FAULT. LOGIC is alert-only.
    @
    @ Undervoltage is only checked while a subsystem is commanded ON and past UV_SETTLE_S,
    @ because the DRIVE/ARM/SCIENCE INA780 VBUS pins sit on the switched side and read ~0 V
    @ whenever the channel is off. Sensor loss (INA invalid reads, MCP9808 FAILURE) is
    @ alert-only and never trips.
    active component FPManager {

        # ----------------------------------------------------------------------
        # State machine instances: one fault latch per monitored subsystem.
        # `drop` on a full queue instead of the default assert (which would reboot the board):
        # FPManager re-sends a fault on every out-of-bounds sample until the latch is set.
        # ----------------------------------------------------------------------

        @ Fault latch for the drivetrain subsystem (any of the six wheel sensors trips all six)
        state machine instance fp_drivetrainSM: FPStateMachine drop

        @ Fault latch for the arm subsystem
        state machine instance fp_armSM: FPStateMachine drop

        @ Fault latch for the science subsystem
        state machine instance fp_scienceSM: FPStateMachine drop

        @ Fault latch for the logic board (alert-only)
        state machine instance fp_logicSM: FPStateMachine drop

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Power (voltage/current) reading from InaManager, once per sensor per poll cycle (10 Hz).
        @ `hook` on overflow: a dropped reading is counted and reported instead of silently lost
        @ or asserting.
        async input port powerReadingIn: Billee.PowerReadingPort hook

        @ Thermal reading from McpManager, once per sensor per poll cycle (the shared arm/science
        @ sensor arrives twice: once tagged ARM, once tagged SCIENCE).
        async input port thermalReadingIn: Billee.ThermalReadingPort hook

        @ Actual power state changes from SubsystemManager (used to gate the undervoltage check).
        @ `hook` on overflow: the change is parked and applied before the next reading, so it is
        @ never lost and never asserts in SubsystemManager's thread.
        async input port powerStateIn: Billee.PowerStateChanged hook

        @ Latches (true) or releases (false) the fault inhibit in SubsystemManager
        output port faultInhibitOut: Billee.FaultInhibit

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Clear a latched fault on a subsystem. Releases the SubsystemManager inhibit but does
        @ not turn anything on. Rejected (EXECUTION_ERROR) while that subsystem's temperature
        @ sensor still reports a FAULT-level temperature.
        async command CLEAR_FAULT(
            subsystem: Billee.Subsystems @< DRIVETRAIN, ARM, SCIENCE or LOGIC
        ) opcode 0x30

        # ----------------------------------------------------------------------
        # Parameters (defaults set by Luca 2026-09-26; 30 A current limits are deliberately high
        # for initial testing). Ids/opcodes start at 0x10 so they can never collide with the
        # old F32 VBUS_FAULT_LOW/HIGH/CURRENT_FAULT_HIGH params (ids 0-2) in a saved param file.
        # ----------------------------------------------------------------------

        @ Drivetrain bounds, 4S pack. currentFaultHigh applies to each wheel sensor individually.
        param DRIVETRAIN_BOUNDS: Billee.PowerBounds \
            default {vbusFaultLow = 13.6, vbusFaultHigh = 17.2, currentFaultHigh = 30.0} \
            id 0x10 \
            set opcode 0x10 \
            save opcode 0x11

        @ Arm bounds, 6S pack
        param ARM_BOUNDS: Billee.PowerBounds \
            default {vbusFaultLow = 21.6, vbusFaultHigh = 25.5, currentFaultHigh = 30.0} \
            id 0x11 \
            set opcode 0x12 \
            save opcode 0x13

        @ Science bounds, 6S pack
        param SCIENCE_BOUNDS: Billee.PowerBounds \
            default {vbusFaultLow = 21.6, vbusFaultHigh = 25.5, currentFaultHigh = 30.0} \
            id 0x12 \
            set opcode 0x14 \
            save opcode 0x15

        @ Logic board bounds, 6S pack (alert-only)
        param LOGIC_BOUNDS: Billee.PowerBounds \
            default {vbusFaultLow = 21.6, vbusFaultHigh = 25.5, currentFaultHigh = 30.0} \
            id 0x13 \
            set opcode 0x16 \
            save opcode 0x17

        @ Seconds after a subsystem is commanded ON before its undervoltage check is armed
        param UV_SETTLE_S: F32 default 1.0 id 0x14 set opcode 0x18 save opcode 0x19

        @ Consecutive out-of-bounds samples (10 Hz) before an overcurrent fault
        param DEBOUNCE_OC: U8 default 3 id 0x15 set opcode 0x1A save opcode 0x1B

        @ Consecutive out-of-bounds samples (10 Hz) before an overvoltage fault
        param DEBOUNCE_OV: U8 default 3 id 0x16 set opcode 0x1C save opcode 0x1D

        @ Consecutive out-of-bounds samples (10 Hz) before an undervoltage fault
        param DEBOUNCE_UV: U8 default 5 id 0x17 set opcode 0x1E save opcode 0x1F

        @ Consecutive FAULT-level thermal samples (1 Hz) before an overtemperature fault
        param DEBOUNCE_THERMAL: U8 default 2 id 0x18 set opcode 0x20 save opcode 0x21

        # ----------------------------------------------------------------------
        # Telemetry: bounds in effect per subsystem
        # ----------------------------------------------------------------------

        @ Voltage/current bounds applied to the drivetrain (each wheel sensor)
        telemetry DRIVETRAIN_POWER_BOUNDS: Billee.PowerBounds id 0x10

        @ Voltage/current bounds applied to the arm subsystem
        telemetry ARM_POWER_BOUNDS: Billee.PowerBounds id 0x11

        @ Voltage/current bounds applied to the science subsystem
        telemetry SCIENCE_POWER_BOUNDS: Billee.PowerBounds id 0x12

        @ Voltage/current bounds applied to the logic board
        telemetry LOGIC_POWER_BOUNDS: Billee.PowerBounds id 0x13

        @ Readings dropped because the FPManager queue was full (should stay 0)
        telemetry FpReadingsDropped: U32 id 0x14

        # ----------------------------------------------------------------------
        # Telemetry: latched fault state per subsystem
        # ----------------------------------------------------------------------

        @ Fault latch state of the drivetrain subsystem
        telemetry DRIVETRAIN_FAULT_STATE: Billee.FaultState id 0

        @ Fault latch state of the arm subsystem
        telemetry ARM_FAULT_STATE: Billee.FaultState id 1

        @ Fault latch state of the science subsystem
        telemetry SCIENCE_FAULT_STATE: Billee.FaultState id 2

        @ Fault latch state of the logic board (alert-only)
        telemetry LOGIC_FAULT_STATE: Billee.FaultState id 3

        # ----------------------------------------------------------------------
        # Telemetry: per-sensor evaluation of the latest reading
        # (NOMINAL = in bounds, TRIPPED = out of bounds this sample, STALE = read failed)
        # ----------------------------------------------------------------------

        @ Latest evaluation of the drivetrain motor 1 INA780B
        telemetry DRIVE1_POWER_STATE: Billee.FaultState id 4

        @ Latest evaluation of the drivetrain motor 2 INA780B
        telemetry DRIVE2_POWER_STATE: Billee.FaultState id 5

        @ Latest evaluation of the drivetrain motor 3 INA780B
        telemetry DRIVE3_POWER_STATE: Billee.FaultState id 6

        @ Latest evaluation of the drivetrain motor 4 INA780B
        telemetry DRIVE4_POWER_STATE: Billee.FaultState id 7

        @ Latest evaluation of the drivetrain motor 5 INA780B
        telemetry DRIVE5_POWER_STATE: Billee.FaultState id 8

        @ Latest evaluation of the drivetrain motor 6 INA780B
        telemetry DRIVE6_POWER_STATE: Billee.FaultState id 9

        @ Latest evaluation of the arm subsystem INA780B
        telemetry ARM_POWER_STATE: Billee.FaultState id 10

        @ Latest evaluation of the science subsystem INA780B
        telemetry SCIENCE_POWER_STATE: Billee.FaultState id 11

        @ Latest evaluation of the logic board INA780B
        telemetry LOGIC_POWER_STATE: Billee.FaultState id 12

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ A subsystem latched a fault and was forced OFF
        event SubsystemFaultShutdown(
            subsystemName: Billee.Subsystems
            reason: Billee.FaultReason
        ) \
            severity warning high \
            id 0 \
            format "Subsystem {} forced OFF by FPManager due to {}; send CLEAR_FAULT to re-arm"

        @ A subsystem's latched fault was cleared by ground command (nothing was turned on)
        event SubsystemFaultCleared(
            subsystemName: Billee.Subsystems
        ) \
            severity activity high \
            id 1 \
            format "Subsystem {} fault cleared (still OFF until commanded ON)"

        @ The logic board latched a fault. Alert-only: FPManager cannot power it off.
        event LogicFaultDetected(
            reason: Billee.FaultReason
        ) \
            severity warning high \
            id 2 \
            format "Logic board fault detected: {} (alert only)"

        @ The logic board fault was cleared by ground command
        event LogicFaultCleared() \
            severity activity high \
            id 3 \
            format "Logic board fault cleared"

        @ Which sensor caused a power fault, and its reading
        event PowerFaultDetail(
            sensorId: Billee.InaSensorId
            reason: Billee.FaultReason
            voltage: F32
            current: F32
        ) \
            severity warning high \
            id 4 \
            format "Power fault on {}: {} ({.2f} V, {.2f} A)"

        @ A temperature sensor reported FAILURE (not readable). Alert only, no trip.
        event ThermalSensorLost(
            sensorId: Billee.McpSensorId
        ) \
            severity warning high \
            id 5 \
            format "Temperature sensor {} not readable (alert only, no trip)"

        @ A previously lost temperature sensor is readable again
        event ThermalSensorRecovered(
            sensorId: Billee.McpSensorId
        ) \
            severity activity high \
            id 6 \
            format "Temperature sensor {} readable again"

        @ CLEAR_FAULT refused because the subsystem's temperature is still at FAULT level
        event FaultClearRejected(
            subsystemName: Billee.Subsystems
        ) \
            severity warning low \
            id 7 \
            format "CLEAR_FAULT for {} rejected: temperature still at FAULT level"

        @ CLEAR_FAULT sent for a subsystem with no latched fault
        event FaultClearNoop(
            subsystemName: Billee.Subsystems
        ) \
            severity activity low \
            id 8 \
            format "CLEAR_FAULT for {}: no fault latched"

        @ The FPManager input queue overflowed and readings were dropped. Fires once per boot;
        @ see FpReadingsDropped for the count.
        event FpReadingDropped() \
            severity warning high \
            id 9 \
            format "FPManager queue full: sensor readings dropped"

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
