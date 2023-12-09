#include "Barometer.hpp"
#include "MissionConstants.hpp"

#include <fstream>
#include <stdexcept>

Barometer::Barometer()
{
    std::ifstream ifstream(MissionConstants::baroPath + "/in_pressure_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");
    ifstream.close();

    ifstream = std::ifstream(MissionConstants::baroPath + "/in_temp_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");
    ifstream.close();
}

double Barometer::GetPressure()
{
    std::ifstream ifstream(MissionConstants::baroPath + "/in_pressure_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");

    double dPressure;
    ifstream >> dPressure;

    ifstream.close();
    return dPressure;
}

double Barometer::GetTemperature()
{
    std::ifstream ifstream(MissionConstants::baroPath + "/in_temp_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");

    double dTemperature;
    ifstream >> dTemperature;

    ifstream.close();
    return dTemperature * 0.001;
}
