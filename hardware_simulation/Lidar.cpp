// hardware_simulation/Lidar.cpp
#include "Lidar.hpp"
#include "UDPClient.hpp"

Lidar::Lidar()
{
    SetUseLidar(true);
}

std::tuple<double> Lidar::GetLidarDistance()
{
    return UDPClient::GetInstance().GetLidarDistance();
}
