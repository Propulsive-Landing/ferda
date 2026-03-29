// hardware_simulation/GPS.cpp
#include "GPS.hpp"
#include "UDPClient.hpp"

GPS::GPS() {}

std::tuple<double, double, double> GPS::GetGPSPosition()
{
    return UDPClient::GetInstance().GetGPSPosition();
}

std::tuple<double, double> GPS::GetGPSVelocity()
{
    return UDPClient::GetInstance().GetGPSVelocity();
}
