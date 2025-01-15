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

std::tuple<double, double, double> IMU::GetBodyAngularRate() {
    return UDPClient::GetInstance().GetBodyAngularRate();
}

std::tuple<double, double, double> IMU::GetBodyAcceleration() {
    return UDPClient::GetInstance().GetBodyAcceleration();
}
