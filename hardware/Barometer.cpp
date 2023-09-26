#include "Barometer.hpp"

#include <fstream>
#include <stdexcept>

Barometer::Barometer()
{
    std::ifstream ifstream(sPressurePath);
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");
    ifstream.close();

    ifstream = std::ifstream(sTemperaturePath);
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");
    ifstream.close();
}

double Barometer::GetPressure()
{
    std::ifstream ifstream(sPressurePath);
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");

    int nPressure;
    ifstream >> nPressure;

    ifstream.close();
    return nPressure;
}

double Barometer::GetTemperature()
{
    std::ifstream ifstream(sTemperaturePath);
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");

    int nTemperature;
    ifstream >> nTemperature;

    ifstream.close();
    return nTemperature;
}
