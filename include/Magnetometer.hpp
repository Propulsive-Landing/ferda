#pragma once

#include <tuple>
#include "IMU.hpp"

class Magnetometer : public IMU
{
public:
    std::tuple<double, double, double> GetMagneticField();
};