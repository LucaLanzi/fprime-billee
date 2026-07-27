module Billee {
    @ Manager for the on-board INA780B current/voltage/power monitors (I2C0)
    active component InaManager {

        @ Output port allowing to connect to an I2c bus driver for writeRead operations to the INA780B sensors
        output port busWriteRead: Drv.I2cWriteRead

        @ Async scheduler input port to poll power data from the sensors
        async input port run: Svc.Sched

        @ Telemetry for the drivetrain motor 1 INA780B (I2C0, 0x40)
        telemetry DRIVE1_POWER: PowerReading id 0

        @ Telemetry for the drivetrain motor 2 INA780B (I2C0, 0x41)
        telemetry DRIVE2_POWER: PowerReading id 1

        @ Telemetry for the drivetrain motor 3 INA780B (I2C0, 0x43)
        telemetry DRIVE3_POWER: PowerReading id 2

        @ Telemetry for the drivetrain motor 4 INA780B (I2C0, 0x44)
        telemetry DRIVE4_POWER: PowerReading id 3

        @ Telemetry for the drivetrain motor 5 INA780B (I2C0, 0x45)
        telemetry DRIVE5_POWER: PowerReading id 4

        @ Telemetry for the drivetrain motor 6 INA780B (I2C0, 0x47)
        telemetry DRIVE6_POWER: PowerReading id 5

        @ Telemetry for the arm subsystem INA780B (I2C0, 0x4C)
        telemetry ARM_POWER: PowerReading id 6

        @ Telemetry for the science subsystem INA780B (I2C0, 0x4D)
        telemetry SCIENCE_POWER: PowerReading id 7

        @ Telemetry for the logic board INA780B (I2C0, 0x4F)
        telemetry LOGIC_POWER: PowerReading id 8

        @ Reports that at least one INA780B read failed during a poll cycle
        event InaReadFailure() \
            severity warning high \
            id 0 \
            format "INA780B power read failed"

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
