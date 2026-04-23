// hardware_simulation/Magnetometer.cpp
#include "Magnetometer.hpp"
#include "UDPClient.hpp"

Magnetometer::Magnetometer()
{
    useMagnometer = true;
}

std::tuple<double, double, double> Magnetometer::GetMagneticField()
{
    return UDPClient::GetInstance().GetMagneticField();
}
