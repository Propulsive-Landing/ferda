#include "Barometer.hpp"

#include <fstream>
#include <stdexcept>

Barometer::Barometer()
{
    std::ifstream ifstream("/sys/bus/iio/devices/iio:device1/in_pressure_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");
    ifstream.close();

    ifstream = std::ifstream("/sys/bus/iio/devices/iio:device1/in_temp_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");
    ifstream.close();
}

double Barometer::GetPressure()
{
    std::ifstream ifstream("/sys/bus/iio/devices/iio:device1/in_pressure_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");

    double nPressure;
    ifstream >> nPressure;

    ifstream.close();
    return nPressure;
}

double Barometer::GetTemperature()
{
    std::ifstream ifstream("/sys/bus/iio/devices/iio:device1/in_temp_input");
    if (!ifstream.is_open())
        throw std::runtime_error("barometer not present");

    double nTemperature;
    ifstream >> nTemperature;

    ifstream.close();
    return nTemperature * 0.001;
}
