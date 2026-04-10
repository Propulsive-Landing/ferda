#include "LoadCell.hpp"
#include <wiringPi.h>

double LoadCell::ReadSensor(int pin)
{
    // Read raw reading for ADS1115
    float rawVal = analogRead(pin);

    // Convert to voltage (0.0 to 5.0 volts)
    float rawVolt = (float)rawVal / 32767 * 6.144;

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
