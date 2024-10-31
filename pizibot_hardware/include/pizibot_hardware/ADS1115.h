#ifndef ADS1115_H
#define ADS1115_H

#include <cstdint>

#define ADS1115_ADDRESS 0x48 // Default address for ADS1115

// Registers and configurations for ADS1115
#define ADS1115_CONVERSION_REG 0x00
#define ADS1115_CONFIG_REG 0x01
#define ADS1115_CONFIG_MUX_SINGLE_0 0x4000 // Single-ended input on AIN0
#define ADS1115_CONFIG_MODE_SINGLE 0x0100  // Single-shot mode
#define ADS1115_CONFIG_DR_128SPS 0x0080    // Data rate of 128 samples per second
#define ADS1115_CONFIG_OS_SINGLE 0x8000    // Start single conversion

#define ADS1115_CONFIG_GAIN_2_048V 0x0400 // Full-scale range ±2.048V
#define ADS1115_CONFIG_GAIN_4_096V 0x0200 // Full-scale range ±4.096V
#define ADS1115_CONFIG_GAIN_6_144V 0x0000 // Full-scale range ±6.144V

class ADS1115 {
public:
    ADS1115(int i2cChannel = 1, uint8_t address = ADS1115_ADDRESS, float voltageScale = 2.048);
    ~ADS1115();

    bool isInitialized() const;
    bool open(); // Function to open the I2C bus
    void close(); // Function to close the I2C bus
    void configure(uint16_t gain = ADS1115_CONFIG_GAIN_2_048V);
    int16_t read() const;
    float convertToVoltage(int16_t value) const;
    float readVoltage() const;

private:
    int i2cChannel;
    uint8_t address;
    int handle;
    float voltageScale;
};

#endif // ADS1115_H
