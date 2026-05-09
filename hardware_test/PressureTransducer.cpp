#include "PressureTransducer.hpp"
#include <iostream>
#include <random>

// For testing, return random values
double PressureTransducer::ReadSensor(int pin, double maxPSI)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, maxPSI * 0.5);
    return dis(gen);
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
        maxPSI = 200.0;
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
