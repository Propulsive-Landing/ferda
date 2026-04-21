#pragma once

#include <tuple>
#include "IMU.hpp"

class Magnetometer : public IMU
{
public:
    Magnetometer();
    std::tuple<double, double, double> GetMagneticField();
    bool getUseMagnometer() { return useMagnometer; }
    void setUseMagnometer(const bool &state) { useMagnometer = state; }

private:
    bool useMagnometer;
};