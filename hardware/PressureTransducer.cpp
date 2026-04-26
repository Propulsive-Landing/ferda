#include "PressureTransducer.hpp"
#include <wiringPi.h>

double PressureTransducer::ReadSensor(int pin, double maxPSI)
{
    // Read raw reading for ADS1115
    int rawVal = analogRead(pin);

    // Convert to voltage (0.0 to 5.0 volts)
    float rawVolt = (float)rawVal / 32767 * 6.144;

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
    case NitrousTankLine:
        pin = MissionConstants::kNitrousTankLinePTPin;
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
