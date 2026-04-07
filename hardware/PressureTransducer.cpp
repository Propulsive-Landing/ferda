#include "PressureTransducer.hpp"
#include <pigpio.h>
#include <algorithm>

// TODO: This function needs to be implemented based on your ADC hardware
// The original Arduino code used analogRead which reads 0-1023 (10-bit)
// For Raspberry Pi, you'll need an external ADC (e.g., MCP3008 via SPI)
// This is a placeholder that matches the original conversion logic
double PressureTransducer::ReadSensor(int pin, double maxPSI)
{
    // Placeholder: Replace with actual ADC reading
    // Example implementation would be:
    // uint16_t rawVal = readADC(pin); // Read from your ADC hardware

    // For now, return 0.0 - this MUST be implemented for actual hardware
    uint16_t rawVal = 0; // TODO: Read from actual ADC

    // Convert to voltage (0.0 to 5.0 volts)
    // Original: (float)rawVal / 204.6 for 0-1023 range mapped to 0-5V
    // For 10-bit ADC: 1023 / 5.0 = 204.6
    float rawVolt = (float)rawVal / 204.6;

    // Clamp voltage to sensor range (0.5V to 4.5V)
    if (rawVolt < 0.5)
    {
        rawVolt = 0.5;
    }
    else if (rawVolt > 4.5)
    {
        rawVolt = 4.5;
    }

    // Normalize from 0.0 to 1.0 (0.5V = 0.0, 4.5V = 1.0)
    float normalized = (rawVolt - 0.5) / (4.5 - 0.5);

    // Convert to PSI
    float psi = normalized * maxPSI;

    return psi;
}

double PressureTransducer::ReadPSI(PTSensor sensor)
{
    int pin;
    double maxPSI = 200.0; // Default for low-pressure sensors

    switch (sensor)
    {
    case NitrogenLine:
        pin = MissionConstants::kNitrogenLinePTPin;
        maxPSI = 1000.0;
        break;
    case EthanolTank:
        pin = MissionConstants::kEthanolTankPTPin;
        maxPSI = 1000.0;
        break;
    case NitrousLine:
        pin = MissionConstants::kNitrousLinePTPin;
        maxPSI = 1000.0;
        break;
    case OxygenLine:
        pin = MissionConstants::kOxygenLinePTPin;
        maxPSI = 200.0; // 0-200 PSI range
        break;
    case FuelInlet:
        pin = MissionConstants::kFuelInletPTPin;
        maxPSI = 1000.0;
        break;
    case FuelOutlet:
        pin = MissionConstants::kFuelOutletPTPin;
        maxPSI = 1000.0;
        break;
    case ChamberPressure:
        pin = MissionConstants::kChamberPressurePTPin;
        maxPSI = 1000.0;
        break;
    default:
        return 0.0;
    }
    return ReadSensor(pin, maxPSI);
}
