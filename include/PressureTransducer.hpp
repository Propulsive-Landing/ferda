#pragma once

#include "MissionConstants.hpp"

class PressureTransducer
{
public:
    enum PTSensor
    {
        NitrogenLine,
        EthanolTank,
        NitrousLine,
        NitrousTankLine,
        OxygenLine,
        FuelInlet,
        FuelOutlet,
        ChamberPressure
    };

    PressureTransducer() = default;
    double ReadPSI(PTSensor sensor); // Returns pressure in PSI

private:
    double ReadSensor(int pin, double maxPSI); // Internal helper
};
