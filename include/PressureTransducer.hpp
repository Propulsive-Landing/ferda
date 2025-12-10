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
        OxygenLine,
        FuelInlet,
        FuelOutlet,
        ChamberPressure
    };

    PressureTransducer() = default;
    double ReadPSI(PTSensor sensor); // Returns pressure in PSI
    double ReadPSI2(PTSensor sensor); // Returns pressure in PSI (0-1000 range for high-pressure sensors)

private:
    double ReadSensor(int pin, double maxPSI); // Internal helper
};

