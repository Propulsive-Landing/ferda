// Update magnetometer class for hardware implementation

// hardware_simulation/Magnetometer.cpp
#include "Magnetometer.hpp"

std::tuple<double, double, double> Magnetometer::GetMagneticField()
{
    return std::make_tuple(0.0, 0.0, 0.0);
}
