module Billee {
    @ Device manager to poll temperature data from the on-board MCP9808 temp sensors
    active component McpManager {

        @ Bind the ThermalStateMachine to McpManager
        state machine instance mcp_thermalStateMachine: ThermalStateMachine

        @ Output port allowing to connect to an I2c bus driver for writeRead operations to the mcp9808 temp sensors
        output port mcpWriteRead: Drv.I2cWriteRead

        @ Async scheduler input port to poll temp data from the sensors
        async input port run: Svc.Sched

        @ Forwards each sensor's thermal reading to FPManager, tagged with the subsystem it
        @ belongs to. The shared arm/science sensor is broadcast twice: once as ARM, once as
        @ SCIENCE.
        output port thermalReadingOut: Billee.ThermalReadingPort

        @ Telemetry for the logic board MCP9808 (I2C1, 0x18)
        telemetry LOGIC_TEMP: ThermalReading id 0

        @ Telemetry for the drivetrain MCP9808 (I2C1, 0x19)
        telemetry DRIVE_TEMP: ThermalReading id 1

        @ Telemetry for the arm/science MCP9808 (I2C1, 0x1A)
        telemetry ARM_SCI_TEMP: ThermalReading id 2

        @ IDLE Low temperature threshold
        param MCP_IDLE_LOW: F32 \
            default 10 \
            id 0x00 \
            set opcode 0x01 \
            save opcode 0x02

        @ IDLE High temperature threshold
        param MCP_IDLE_HIGH: F32 \
            default 60 \
            id 0x01 \
            set opcode 0x03 \
            save opcode 0x04

        @ WARNING Low temperature threshold
        param MCP_WARN_LOW: F32 \
            default -20 \
            id 0x02 \
            set opcode 0x05 \
            save opcode 0x06

        @ WARNING High temperature threshold
        param MCP_WARN_HIGH: F32 \
            default 80 \
            id 0x03 \
            set opcode 0x07 \
            save opcode 0x08

        @ FAULT Low temperature threshold
        param MCP_FAULT_LOW: F32 \
            default -40 \
            id 0x04 \
            set opcode 0x09 \
            save opcode 0x10

        @ FAULT High temperature threshold
        param MCP_FAULT_HIGH: F32 \
            default 100 \
            id 0x05 \
            set opcode 0x11 \
            save opcode 0x12

        @ Telemetry for IDLE state low threshold
        telemetry MCP_IDLE_LOW: F32 id 0x10

        @ Telemetry for IDLE state high threshold
        telemetry MCP_IDLE_HIGH: F32 id 0x11

        @ Telemetry for WARNING state low threshold
        telemetry MCP_WARN_LOW: F32 id 0x12

        @ Telemetry for WARNING state high threshold
        telemetry MCP_WARN_HIGH: F32 id 0x13

        @ Telemetry for FAULT state low threshold
        telemetry MCP_FAULT_LOW: F32 id 0x14

        @ Telemetry for FAULT state high threshold
        telemetry MCP_FAULT_HIGH: F32 id 0x15

        @ Reports that at least one MCP9808 read failed. Only fires once on the transition
        @ into a failed state, not on every poll cycle the sensors remain disconnected.
        event McpReadFailure() \
            severity warning high \
            id 0 \
            format "MCP9808 temperature read failed"

        @ Reports that all MCP9808 sensors are reading successfully again after a prior failure
        event McpReadRecovered() \
            severity activity high \
            id 1 \
            format "MCP9808 temperature read recovered"

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
