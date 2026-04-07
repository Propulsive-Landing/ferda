// Update magnetometer class for hardware implementation

// hardware_simulation/Magnetometer.cpp
#include "Magnetometer.hpp"
#include "MissionConstants.hpp"
#include "cmath"


std::tuple<double, double, double>  Magnetometer::GetMagneticField()
{
    double nMagX = (double)read16LE(fd, MissionConstants::REG_MAG_X) / 16.0f;
    double nMagY = (double)read16LE(fd, MissionConstants::REG_MAG_Y) / 16.0f;
    double nMagZ = (double)read16LE(fd, MissionConstants::REG_MAG_Z) / 16.0f;
    
    return std::make_tuple(nMagX, nMagY, nMagZ);

}
