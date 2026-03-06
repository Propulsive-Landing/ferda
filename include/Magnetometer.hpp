#pragma once

#include <tuple>
#include "IMU.hpp"

class Magnetometer ::public IMU
{
public:
    Magnetometer();
    std::tuple<double, double, double> GetMagneticField();
    std::tuple<double> MagnetometerAvailable();
};