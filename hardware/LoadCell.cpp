#include "LoadCell.hpp"
#include <pigpio.h>

// TODO: This function needs to be implemented based on your ADC hardware
// The original Arduino code used analogRead
// For Raspberry Pi, you'll need an external ADC (e.g., MCP3008 via SPI)
double LoadCell::ReadSensor(int pin)
{
    // Placeholder: Replace with actual ADC reading
    uint16_t rawVal = 0; // TODO: Read from actual ADC
    
    // Convert to voltage (0.0 to 5.0 volts)
    float rawVolt = (float)rawVal / 204.6;
    
    // Normalize from 0.0 to 1.0 (0.0V = 0kg, 5.0V = 1000kg)
    float normalized = rawVolt / 5.0;
    
    // Convert to kg, then to lbs
    float kg = normalized * 1000.0;
    float lbs = kg * 2.20462;
    
    return lbs;
}

double LoadCell::ReadLBS()
{
    return ReadSensor(MissionConstants::kLoadCellPin);
}

