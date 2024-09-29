// hardware_simulation/IMU.cpp
#include "IMU.hpp"
#include "UDPClient.hpp"

IMU::IMU() {}

std::tuple<double, double, double> IMU::GetBodyAngularRate() {
    return UDPClient::GetInstance().GetBodyAngularRate();
}

std::tuple<double, double, double> IMU::GetBodyAcceleration() {
    return UDPClient::GetInstance().GetBodyAcceleration();
}