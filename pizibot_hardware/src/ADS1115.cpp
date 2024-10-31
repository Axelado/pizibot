#include "pizibot_hardware/ADS1115.h"
#include <iostream>
#include <pigpio.h>
#include <unistd.h>

// Constructor: Initializes ADS1115 with I2C and pigpio setup
ADS1115::ADS1115(int i2cChannel, uint8_t address, float voltageScale)
    : i2cChannel(i2cChannel), address(address), voltageScale(voltageScale) {}

// Destructor: Closes I2C connection and terminates pigpio
ADS1115::~ADS1115()
{
    i2cClose(handle);
    gpioTerminate();
}

bool ADS1115::open()
{
    if (gpioInitialise() < 0)
    {
        std::cerr << "Error initializing pigpio" << std::endl;
        return false;
    }

    handle = i2cOpen(1, address, 0);
    if (handle < 0)
    {
        std::cerr << "Error opening I2C bus" << std::endl;
        return false;
    }

    return true;
}

void ADS1115::close()
{
    if (handle >= 0)
    {
        i2cClose(handle);
    }
    gpioTerminate();
}

// Configures ADS1115 with gain and other settings
void ADS1115::configure(uint16_t gain)
{
    int config = ADS1115_CONFIG_OS_SINGLE | ADS1115_CONFIG_MUX_SINGLE_0 |
                 gain | ADS1115_CONFIG_MODE_SINGLE | ADS1115_CONFIG_DR_128SPS;

    i2cWriteWordData(handle, ADS1115_CONFIG_REG, __builtin_bswap16(config));
}

// Reads the raw conversion value from ADS1115
int16_t ADS1115::read() const
{
    int16_t result = i2cReadWordData(handle, ADS1115_CONVERSION_REG);
    return __builtin_bswap16(result);
}

// Converts raw analog value to voltage
float ADS1115::convertToVoltage(int16_t value) const
{
    return (value * voltageScale) / 32768.0;
}

float ADS1115::readVoltage() const
{
    int16_t value = read();
    return convertToVoltage(value);
}