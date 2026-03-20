#pragma once

#include <tuple>

class GPS
{
public:
    GPS();
    std::tuple<double, double, double> GetGPSPosition();
    std::tuple<double, double> GetGPSVelocity();
    std::tuple<double> GPSAvailable();
};