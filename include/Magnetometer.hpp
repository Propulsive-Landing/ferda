#pragma once

#include <tuple>

class Magnetometer
{
public:
    Magnetometer();
    std::tuple<double, double, double> GetMagneticField();
};