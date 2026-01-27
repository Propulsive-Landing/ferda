#pragma once

#include <tuple>

class GPS
{
public:
    GPS();
    std::tuple<double, double, double> GetPosition();
};