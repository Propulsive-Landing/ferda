#include <tuple>

#include "IMU.hpp"

std::tuple<double, double, double> IMU::GetBodyAngularRate()
{
    return std::make_tuple(0.0,0.0,0.0);
}

std::tuple<double, double, double> IMU::GetBodyAcceleration()
{
    return std::make_tuple(0.0,0.0,0.0);
}
