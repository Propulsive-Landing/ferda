#include "Barometer.hpp"
#include "MissionConstants.hpp"

#include <fstream>
#include <stdexcept>

#include <string>

namespace {
    std::string baroPath = "/home/pi/barometer_device";
}

Barometer::Barometer()
{
    std::ifstream ifstream(baroPath + "/in_pressure_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");
    ifstream.close();

    ifstream = std::ifstream(baroPath + "/in_temp_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");
    ifstream.close();
}

double Barometer::GetPressure()
{
    std::ifstream ifstream(baroPath + "/in_pressure_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");

    double dPressure;
    ifstream >> dPressure;

    ifstream.close();
    return dPressure;
}

double Barometer::GetTemperature()
{
    std::ifstream ifstream(baroPath + "/in_temp_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");

    double dTemperature;
    ifstream >> dTemperature;

    ifstream.close();
    return dTemperature * 0.001;
}
