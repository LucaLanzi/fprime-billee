// ======================================================================
// \title  InaManager.cpp
// \brief  cpp file for InaManager component implementation class
// ======================================================================

#include "Components/InaManager/InaManager.hpp"

namespace Billee {

namespace {
// INA780B conversion factors (datasheet Table 7-1 / Section 7.5.1)
constexpr F32 VBUS_LSB_VOLTS = 0.003125f;    // 3.125 mV/LSB
constexpr F32 CURRENT_LSB_AMPS = 0.0024f;    // 2.4 mA/LSB
constexpr F32 POWER_LSB_WATTS = 0.00048f;    // 480 uW/LSB
}  // namespace

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

InaManager ::InaManager(const char* const compName) : InaManagerComponentBase(compName) {
    deviceAddrs[0] = DRIVE1_ADDR;
    deviceAddrs[1] = DRIVE2_ADDR;
    deviceAddrs[2] = DRIVE3_ADDR;
    deviceAddrs[3] = DRIVE4_ADDR;
    deviceAddrs[4] = DRIVE5_ADDR;
    deviceAddrs[5] = DRIVE6_ADDR;
    deviceAddrs[6] = ARM_ADDR;
    deviceAddrs[7] = SCIENCE_ADDR;
    deviceAddrs[8] = LOGIC_ADDR;

    subsystemForIndex[0] = Billee::Subsystems::DRIVETRAIN;
    subsystemForIndex[1] = Billee::Subsystems::DRIVETRAIN;
    subsystemForIndex[2] = Billee::Subsystems::DRIVETRAIN;
    subsystemForIndex[3] = Billee::Subsystems::DRIVETRAIN;
    subsystemForIndex[4] = Billee::Subsystems::DRIVETRAIN;
    subsystemForIndex[5] = Billee::Subsystems::DRIVETRAIN;
    subsystemForIndex[6] = Billee::Subsystems::ARM;
    subsystemForIndex[7] = Billee::Subsystems::SCIENCE;
    subsystemForIndex[8] = Billee::Subsystems::LOGIC;

    sensorIdForIndex[0] = Billee::InaSensorId::DRIVE1;
    sensorIdForIndex[1] = Billee::InaSensorId::DRIVE2;
    sensorIdForIndex[2] = Billee::InaSensorId::DRIVE3;
    sensorIdForIndex[3] = Billee::InaSensorId::DRIVE4;
    sensorIdForIndex[4] = Billee::InaSensorId::DRIVE5;
    sensorIdForIndex[5] = Billee::InaSensorId::DRIVE6;
    sensorIdForIndex[6] = Billee::InaSensorId::ARM;
    sensorIdForIndex[7] = Billee::InaSensorId::SCIENCE;
    sensorIdForIndex[8] = Billee::InaSensorId::LOGIC;
}

InaManager ::~InaManager() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void InaManager ::run_handler(FwIndexType portNum, U32 context) {
    if (this->m_justBooted) {
        this->m_justBooted = false;
        this->m_startTime = this->getTime().getSeconds();
    }
    const U32 timestamp = this->getTime().getSeconds() - this->m_startTime;

    bool anyFailed = false;
    for (U8 i = 0; i < NUM_SENSORS; i++) {
        Billee::PowerReading& reading = this->m_powerReadings[i];
        reading.set_sourceId(this->sensorIdForIndex[i]);
        reading.set_timestamp(timestamp);

        if (!this->readSensor(this->deviceAddrs[i], reading)) {
            anyFailed = true;
        }
        // Always publish, even on failure: `reading` retains its last-known (or zeroed, if this
        // is the first-ever read) values, so the channel/FPManager never silently goes stale.
        this->writeTelemetry(i, reading);
        this->powerReadingOut_out(0, this->subsystemForIndex[i], reading);
    }

    if (anyFailed) {
        if (!this->m_wasFailed) {
            this->m_wasFailed = true;
            this->log_WARNING_HI_InaReadFailure();
        }
    } else if (this->m_wasFailed) {
        this->m_wasFailed = false;
        this->log_ACTIVITY_HI_InaReadRecovered();
    }
}

// ----------------------------------------------------------------------
// Helper functions
// ----------------------------------------------------------------------

bool InaManager ::readRegister16(U8 deviceAddr, U8 registerAddr, U16& value) {
    U8 writeData[1] = {registerAddr};
    U8 readData[2] = {0, 0};
    Fw::Buffer writeBuffer(writeData, sizeof(writeData));
    Fw::Buffer readBuffer(readData, sizeof(readData));

    const Drv::I2cStatus status = this->busWriteRead_out(0, deviceAddr, writeBuffer, readBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        return false;
    }

    value = static_cast<U16>((static_cast<U16>(readData[0]) << 8) | static_cast<U16>(readData[1]));
    return true;
}

bool InaManager ::readRegister24(U8 deviceAddr, U8 registerAddr, U32& value) {
    U8 writeData[1] = {registerAddr};
    U8 readData[3] = {0, 0, 0};
    Fw::Buffer writeBuffer(writeData, sizeof(writeData));
    Fw::Buffer readBuffer(readData, sizeof(readData));

    const Drv::I2cStatus status = this->busWriteRead_out(0, deviceAddr, writeBuffer, readBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        return false;
    }

    value = (static_cast<U32>(readData[0]) << 16) | (static_cast<U32>(readData[1]) << 8) |
            static_cast<U32>(readData[2]);
    return true;
}

bool InaManager ::readSensor(U8 deviceAddr, Billee::PowerReading& reading) {
    // On a floating/unpopulated I2C0 bus, i2c_write_read() has been observed to falsely report
    // success (with all-zero register content) instead of NACKing like a genuinely absent device
    // should. MANUFACTURER_ID is a fixed, read-only "TI" ASCII value baked into the silicon, so
    // checking it catches that case regardless of what the I2C status code claims.
    U16 manufacturerId = 0;
    if (!this->readRegister16(deviceAddr, REG_MANUFACTURER_ID, manufacturerId) ||
        manufacturerId != EXPECTED_MANUFACTURER_ID) {
        return false;
    }

    U16 rawVbus = 0;
    U16 rawCurrent = 0;
    U32 rawPower = 0;

    if (!this->readRegister16(deviceAddr, REG_VBUS, rawVbus)) {
        return false;
    }
    if (!this->readRegister16(deviceAddr, REG_CURRENT, rawCurrent)) {
        return false;
    }
    if (!this->readRegister24(deviceAddr, REG_POWER, rawPower)) {
        return false;
    }

    reading.set_voltage(static_cast<F32>(rawVbus) * VBUS_LSB_VOLTS);
    reading.set_current(static_cast<F32>(static_cast<I16>(rawCurrent)) * CURRENT_LSB_AMPS);
    reading.set_power(static_cast<F32>(rawPower) * POWER_LSB_WATTS);
    return true;
}

void InaManager ::writeTelemetry(U8 index, const Billee::PowerReading& reading) {
    switch (index) {
        case 0:
            this->tlmWrite_DRIVE1_POWER(reading);
            break;
        case 1:
            this->tlmWrite_DRIVE2_POWER(reading);
            break;
        case 2:
            this->tlmWrite_DRIVE3_POWER(reading);
            break;
        case 3:
            this->tlmWrite_DRIVE4_POWER(reading);
            break;
        case 4:
            this->tlmWrite_DRIVE5_POWER(reading);
            break;
        case 5:
            this->tlmWrite_DRIVE6_POWER(reading);
            break;
        case 6:
            this->tlmWrite_ARM_POWER(reading);
            break;
        case 7:
            this->tlmWrite_SCIENCE_POWER(reading);
            break;
        case 8:
            this->tlmWrite_LOGIC_POWER(reading);
            break;
        default:
            break;
    }
}

}  // namespace Billee
