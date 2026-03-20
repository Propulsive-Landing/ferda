// hardware_simulation/Lidar.cpp
#include "Lidar.hpp"
#include "UDPClient.hpp"

Lidar::Lidar() {}

std::tuple<double> Lidar::GetLidarDistance()
{
    return UDPClient::GetInstance().GetLidarDistance();
}

std::tuple<double> Lidar::LidarAvailable()
{
    return UDPClient::GetInstance().GetLidarAvailable();
}
