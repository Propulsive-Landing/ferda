// Update GPS class for hardware implementation

// hardware_simulation/GPS.cpp
#include "GPS.hpp"

GPS::GPS() {}

std::tuple<double, double, double> GPS::GetGPSPosition()
{
    return std::make_tuple(0.0, 0.0, 0.0);
}

std::tuple<double, double> GPS::GetGPSVelocity()
{
    return std::make_tuple(0.0, 0.0);
}
