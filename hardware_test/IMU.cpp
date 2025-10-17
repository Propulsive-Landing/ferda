#include <tuple>

#include "IMU.hpp"

IMU::IMU()
{
}

std::tuple<double, double, double> IMU::GetBodyAngularRate()
{
    return std::make_tuple(1, 0.0, 0.0);
}

std::tuple<double, double, double> IMU::GetBodyAcceleration()
{
    return std::make_tuple(0, 0, 10.0);
}

void IMU::SetGyroBiasX(double x)
{
    gyroBiasX = x;
}
void IMU::SetGyroBiasY(double y)
{
    gyroBiasY = y;
}
void IMU::SetGyroBiasZ(double z)
{
    gyroBiasZ = z;
}

void IMU::SetAccelBiasX(double x)
{
    accelBiasX = x;
}
void IMU::SetAccelBiasY(double y)
{
    accelBiasY = y;
}
void IMU::SetAccelBiasZ(double z)
{
    accelBiasZ = z;
}