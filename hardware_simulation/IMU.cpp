// hardware_simulation/IMU.cpp
#include "IMU.hpp"
#include "UDPClient.hpp"

IMU::IMU() {}

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

std::tuple<double, double, double> IMU::GetBodyAngularRate()
{
    return UDPClient::GetInstance().GetBodyAngularRate();
}

std::tuple<double, double, double> IMU::GetBodyAcceleration()
{
    return UDPClient::GetInstance().GetBodyAcceleration();
}
