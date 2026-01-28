// hardware_simulation/Magnetometer.cpp
#include "Magnetometer.hpp"
#include "UDPClient.hpp"

Magnetometer::Magnetometer() {}

std::tuple<double, double, double> Magnetometer::GetMagneticField()
{
    return UDPClient::GetInstance().GetMagneticField();
}

std::tuple<double> Magnetometer::MagnetometerAvailable()
{
    return UDPClient::GetInstance().GetMagnetometerAvailable();
}
