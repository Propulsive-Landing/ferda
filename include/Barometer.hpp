#pragma once

#include <cstdint>

class Barometer
{
public:
    Barometer();

    double GetPressure();
    double GetTemperature();
};
