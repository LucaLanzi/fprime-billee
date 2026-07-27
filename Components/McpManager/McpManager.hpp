// ======================================================================
// \title  McpManager.hpp
// \brief  hpp file for McpManager component implementation class
// ======================================================================

#ifndef Billee_McpManager_HPP
#define Billee_McpManager_HPP

#include "Components/McpManager/McpManagerComponentAc.hpp"

namespace Billee {

class McpManager final : public McpManagerComponentBase {
    // Device address and target register address for the on-board MCP9808 sensors
  public:
    static constexpr U8 LOGIC_TEMP_ADDR = 0x18;    //!< I2C address for the logic board temperature sensor
    static constexpr U8 DRIVE_TEMP_ADDR = 0x19;    //!< I2C address for the drivetrain temperature sensor
    static constexpr U8 ARM_SCI_TEMP_ADDR = 0x1A;  //!< I2C address for the arm/science temperature sensor
    U8 deviceAddrs[3];                             //!< Array of device addresses for iterating through sensors

    static constexpr U8 TEMP_REG_ADDR = 0x05;  //!< Register address for temperature data

  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct McpManager object
    McpManager(const char* const compName  //!< The component name
    );

    //! Destroy McpManager object
    ~McpManager();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run
    //!
    //! Async scheduler input port to poll temp data from the sensors
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

  private:
    /* Implementation-specific members */
    Billee::ThermalReading m_thermalReadings[3];  //!< The 3 thermal readings to be logged to telemetry

    /* Determines whether the device has just booted, valid parameter values, and read fail state */
    bool m_justBooted;
    bool m_successfulRead;  // Flag to track whether the most recent read was successful, used to determine
                             // state machine transitions
    bool m_sensorOk[3] = {false, false, false};  // Per-sensor result of the most recent read, so doEvaluate/
                                                  // doReadFail can tell which sensors failed this cycle
    bool m_wasFailed = false;  // Latches so McpReadFailure/McpReadRecovered fire once per transition,
                               // instead of every poll cycle the sensors remain disconnected
    U32 m_startTime = 0;
    Fw::ParamValid m_paramIsValid = Fw::ParamValid::VALID;

    /* Telemetry values for temperature thresholds */
    F32 IDLE_LOW_THR;
    F32 IDLE_HIGH_THR;
    F32 WARN_LOW_THR;
    F32 WARN_HIGH_THR;
    F32 FAULT_LOW_THR;
    F32 FAULT_HIGH_THR;

  private:
    // ----------------------------------------------------------------------
    // Implementations for internal state machine actions
    // ----------------------------------------------------------------------

    //! Implementation for action doRead of state machine Billee_ThermalStateMachine
    //!
    //! Read the temp values from the device
    void Billee_ThermalStateMachine_action_doRead(SmId smId,  //!< The state machine id
                                                   Billee_ThermalStateMachine::Signal signal  //!< The signal
                                                   ) override;

    //! Implementation for action doEvaluate of state machine Billee_ThermalStateMachine
    //!
    //! Evaluate the temp values against thresholds and update telemetry
    void Billee_ThermalStateMachine_action_doEvaluate(SmId smId,  //!< The state machine id
                                                       Billee_ThermalStateMachine::Signal signal  //!< The signal
                                                       ) override;

    //! Implementation for action doReadFail of state machine Billee_ThermalStateMachine
    //!
    //! Log a read failure event
    void Billee_ThermalStateMachine_action_doReadFail(SmId smId,  //!< The state machine id
                                                       Billee_ThermalStateMachine::Signal signal  //!< The signal
                                                       ) override;

    // ----------------------------------------------------------------------
    // Implementations for parameters update
    // ----------------------------------------------------------------------

    //! Handler implementation for parameter updates, used to update threshold values when parameters are updated
    void parameterUpdated(FwPrmIdType id) override;

    // ----------------------------------------------------------------------
    // Class helper functions
    // ----------------------------------------------------------------------

    //! Read temperature from a given I2C device address. Returns false (and leaves temperature at 0) on failure.
    bool readTemp(U8 deviceAddr, F32& temperature);

    //! Determine the temperature state (IDLE, WARN, FAULT) based on the temperature in Celsius
    Billee::ThermalStates determineTempState(F32 tempCelsius);

    //! Write all 3 thermal telemetry channels and forward them to FPManager. Called from both
    //! doEvaluate (all sensors read successfully) and doReadFail (at least one sensor failed) so
    //! the channels are always populated, using tempState FAILURE for any sensor that failed
    //! this cycle.
    void publishReadings();
};

}  // namespace Billee

#endif
