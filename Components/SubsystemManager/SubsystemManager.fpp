module Billee {

    @ Controls and monitors the switched rover subsystems (DRIVETRAIN, ARM, SCIENCE).
    @
    @ Safety rules enforced here (Rev H 11.4):
    @  - Power-ON is refused while the E-STOP is engaged, while its state is unknown, or while
    @    FPManager holds a fault inhibit on that subsystem. Power-OFF is always accepted.
    @  - When the E-STOP engages, every subsystem is forced OFF in software as well. Releasing
    @    the E-STOP restores nothing; each subsystem must be commanded ON again.
    @  - Every power-state change goes through one path and is reported to FPManager.
    active component SubsystemManager {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Input port invoked by the rate group (10 Hz): polls the E-STOP line and publishes state
        async input port run: Svc.Sched

        @ FPManager latches (true) or releases (false) a fault inhibit on a subsystem.
        @ Bypasses the ground-command path so fault protection isn't gated on cmdDisp/GDS.
        async input port faultInhibitIn: Billee.FaultInhibit

        @ Reports every actual power-state change to FPManager
        output port powerStateOut: Billee.PowerStateChanged

        @ GPIO ports controlling the six drivetrain motors
        output port Drive1Set: Drv.GpioWrite
        output port Drive2Set: Drv.GpioWrite
        output port Drive3Set: Drv.GpioWrite
        output port Drive4Set: Drv.GpioWrite
        output port Drive5Set: Drv.GpioWrite
        output port Drive6Set: Drv.GpioWrite

        @ GPIO pin controlling the arm subsystem
        output port ArmSet: Drv.GpioWrite

        @ GPIO pin controlling the science subsystem
        output port ScienceSet: Drv.GpioWrite

        @ GPIO pin reading the E-STOP status line (LOW = E-STOP engaged, HIGH = released)
        output port EStopRead: Drv.GpioRead

        # ----------------------------------------------------------------------
        # Commands (opcode 2 was SET_AUX_POWER_STATE; removed, there is no AUX hardware)
        # ----------------------------------------------------------------------

        @ Set the drivetrain subsystem power state (all six wheels together)
        async command SET_DRIVETRAIN_POWER_STATE(
            driveState: Fw.On @< Requested power state
        ) opcode 0

        @ Set the arm subsystem power state
        async command SET_ARM_POWER_STATE(
            armState: Fw.On @< Requested power state
        ) opcode 1

        @ Set the science subsystem power state
        async command SET_SCIENCE_POWER_STATE(
            scienceState: Fw.On @< Requested power state
        ) opcode 3

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ Reports a subsystem power-state change
        event SubsystemPowerModeEvent(
            subsystemName: Billee.Subsystems
            powerState: Fw.On
        ) \
            severity activity high \
            id 0 \
            format "Subsystem {} power state set to {}"

        @ The E-STOP line read LOW (engaged). Fires once per transition.
        event EStopEngaged() \
            severity warning high \
            id 1 \
            format "E-STOP engaged"

        @ The E-STOP line read HIGH (released). Fires once per transition. Nothing is re-enabled.
        event EStopReleased() \
            severity activity high \
            id 2 \
            format "E-STOP released (subsystems stay OFF until commanded ON)"

        @ The E-STOP line could not be read; it is treated as engaged. Fires once per transition.
        event EStopReadFailure() \
            severity warning high \
            id 3 \
            format "E-STOP status read failed; treating E-STOP as engaged"

        @ The E-STOP engaged while subsystems were ON; all were forced OFF.
        event EStopForcedOff() \
            severity warning high \
            id 4 \
            format "E-STOP engaged: all subsystems forced OFF"

        @ A power-ON command was refused
        event PowerOnRejected(
            subsystemName: Billee.Subsystems
            reason: Billee.PowerOnRejectReason
        ) \
            severity warning low \
            id 5 \
            format "Power ON for {} rejected: {}"

        @ FPManager latched a fault inhibit on a subsystem (forced OFF, ON blocked)
        event FaultInhibitSet(
            subsystemName: Billee.Subsystems
        ) \
            severity warning high \
            id 6 \
            format "Fault inhibit set on {}: forced OFF until fpManager.CLEAR_FAULT"

        @ FPManager released a fault inhibit (nothing is turned on)
        event FaultInhibitCleared(
            subsystemName: Billee.Subsystems
        ) \
            severity activity high \
            id 7 \
            format "Fault inhibit cleared on {} (still OFF until commanded ON)"

        @ A GPIO write failed while applying a power state; the subsystem was driven OFF
        event PowerWriteFailed(
            subsystemName: Billee.Subsystems
        ) \
            severity warning high \
            id 8 \
            format "GPIO write failed while switching {}; all its enables driven OFF"

        # ----------------------------------------------------------------------
        # Telemetry (explicit ids keep E_STOP_Status on its pre-change id)
        # ----------------------------------------------------------------------

        @ Current power state of the drivetrain subsystem
        telemetry DrivetrainPowerState: Fw.On id 0

        @ Current power state of the arm subsystem
        telemetry ArmPowerState: Fw.On id 1

        @ Current power state of the science subsystem
        telemetry SciencePowerState: Fw.On id 2

        @ Current E-STOP status (ON = engaged or unreadable, OFF = released)
        telemetry E_STOP_Status: Fw.On id 4

        # ----------------------------------------------------------------------
        # Standard F Prime ports
        # ----------------------------------------------------------------------

        @ Port for requesting the current time
        time get port timeCaller

        @ Enables command handling
        import Fw.Command

        @ Enables event handling
        import Fw.Event

        @ Enables telemetry channel handling
        import Fw.Channel

        @ Port for retrieving parameter values
        param get port prmGetOut

        @ Port for setting parameter values
        param set port prmSetOut

    }

}
