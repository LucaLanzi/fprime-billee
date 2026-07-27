// ======================================================================
// \title  InaManager.hpp
// \brief  hpp file for InaManager component implementation class
// ======================================================================

#ifndef Billee_InaManager_HPP
#define Billee_InaManager_HPP

#include "Components/InaManager/InaManagerComponentAc.hpp"

namespace Billee {

class InaManager final : public InaManagerComponentBase {
  public:
    static constexpr U8 NUM_SENSORS = 9;

    // I2C addresses (see boards/rpi_pico2_rp2350a_m33.overlay: I2C0, A0/A1 strap table
    // cross-checked directly against the INA780x datasheet's Table 6-2)
    static constexpr U8 DRIVE1_ADDR = 0x40;   //!< A1=GND, A0=GND
    static constexpr U8 DRIVE2_ADDR = 0x41;   //!< A1=GND, A0=VS
    static constexpr U8 DRIVE3_ADDR = 0x43;   //!< A1=GND, A0=SCL
    static constexpr U8 DRIVE4_ADDR = 0x44;   //!< A1=VS, A0=GND
    static constexpr U8 DRIVE5_ADDR = 0x45;   //!< A1=VS, A0=VS
    static constexpr U8 DRIVE6_ADDR = 0x47;   //!< A1=VS, A0=SCL
    static constexpr U8 ARM_ADDR = 0x4C;      //!< A1=SCL, A0=GND
    static constexpr U8 SCIENCE_ADDR = 0x4D;  //!< A1=SCL, A0=VS
    static constexpr U8 LOGIC_ADDR = 0x4F;    //!< A1=SCL, A0=SCL

    U8 deviceAddrs[NUM_SENSORS];  //!< Array of device addresses for iterating through sensors

    // INA780B register addresses (datasheet Table 7-5)
    static constexpr U8 REG_VBUS = 0x05;     //!< 16-bit, 3.125 mV/LSB
    static constexpr U8 REG_DIETEMP = 0x06;  //!< 16-bit (top 12 bits used), 125 m°C/LSB
    static constexpr U8 REG_CURRENT = 0x07;  //!< 16-bit signed, 2.4 mA/LSB
    static constexpr U8 REG_POWER = 0x08;    //!< 24-bit unsigned, 480 uW/LSB

  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct InaManager object
    InaManager(const char* const compName  //!< The component name
    );

    //! Destroy InaManager object
    ~InaManager();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run
    //!
    //! Async scheduler input port to poll power data from the sensors
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

  private:
    /* Implementation-specific members */
    Billee::PowerReading m_powerReadings[NUM_SENSORS];  //!< The readings to be logged to telemetry

    bool m_justBooted = true;
    U32 m_startTime = 0;

  private:
    // ----------------------------------------------------------------------
    // Helper functions
    // ----------------------------------------------------------------------

    //! Read a 16-bit register. Returns false (buffer untouched) on I2C failure.
    bool readRegister16(U8 deviceAddr, U8 registerAddr, U16& value);

    //! Read the 24-bit POWER register. Returns false (buffer untouched) on I2C failure.
    bool readRegister24(U8 deviceAddr, U8 registerAddr, U32& value);

    //! Read voltage/current/power from a single INA780B and populate the reading.
    //! Returns false (reading left unmodified except sourceId/timestamp) if any register read fails.
    bool readSensor(U8 deviceAddr, Billee::PowerReading& reading);

    //! Write the telemetry channel for a given sensor index
    void writeTelemetry(U8 index, const Billee::PowerReading& reading);
};

}  // namespace Billee

#endif
