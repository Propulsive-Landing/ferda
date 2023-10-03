#include <tuple>

#include "IMU.hpp"

tuple<double, double, double> IMU::GetBodyAngularRate()
{
    return make_tuple(0.0,0.0,0.0);
}

tuple<double, double, double> IMU::GetBodyAcceleration()
{
    return make_tuple(0.0,0.0,0.0);
}