// hardware_simulation/GPS.cpp
#include "GPS.hpp"
#include "UDPClient.hpp"

GPS::GPS() : update_count(0) {}

GPS::~GPS() {}

void GPS::Update()
{
    ++update_count;
}

std::tuple<double, double, double> GPS::GetGPSPosition()
{
    return UDPClient::GetInstance().GetGPSPosition();
}

std::tuple<double, double> GPS::GetGPSVelocity()
{
    return UDPClient::GetInstance().GetGPSVelocity();
}

bool GPS::GPSAvailable()
{
    return true; 
}

uint64_t GPS::GetUpdateCount() const
{
    return update_count;
}