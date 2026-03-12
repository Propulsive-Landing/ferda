// hardware_simulation/Magnetometer.cpp
#include "Magnetometer.hpp"
#include "UDPClient.hpp"

std::tuple<double, double, double> Magnetometer::GetMagneticField()
{
    return UDPClient::GetInstance().GetMagneticField();
}
