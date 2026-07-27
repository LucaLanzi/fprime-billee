module Billee {

    @ A component designed to control and monitor the state of the rover subsystems.
    active component SubsystemManager {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Input port invoked by the rate group
        async input port run: Svc.Sched

        @ Commanded by FPManager to power off a subsystem that tripped a fault. Bypasses
        @ the normal ground-command path so fault protection isn't gated on cmdDisp/GDS.
        async input port emergencyPowerOff: Billee.SetPowerState

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

        @ GPIO pin controlling the auxiliary subsystem
        output port AuxSet: Drv.GpioWrite

        @ GPIO pin reading the E-STOP status input (LOW = on, HIGH = off)
        output port EStopRead: Drv.GpioRead

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Set the drivetrain subsystem power state
        async command SET_DRIVETRAIN_POWER_STATE(
            driveState: Fw.On @< Requested power state
        ) opcode 0

        @ Set the arm subsystem power state
        async command SET_ARM_POWER_STATE(
            armState: Fw.On @< Requested power state
        ) opcode 1

        @ Set the auxiliary subsystem power state
        async command SET_AUX_POWER_STATE(
            auxState: Fw.On @< Requested power state
        ) opcode 2

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

        @ Reports the E-STOP status input first being read as HIGH (off)
        event EStopFirstHighEvent() \
            severity activity high \
            id 1 \
            format "E-STOP status first read as HIGH (off)"

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Current power state of the drivetrain subsystem
        telemetry DrivetrainPowerState: Fw.On

        @ Current power state of the arm subsystem
        telemetry ArmPowerState: Fw.On

        @ Current power state of the science subsystem
        telemetry SciencePowerState: Fw.On

        @ Current power state of the auxiliary subsystem
        telemetry AuxPowerState: Fw.On

        @ Current E-STOP status (ON = pulled low, OFF = high)
        telemetry E_STOP_Status: Fw.On

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