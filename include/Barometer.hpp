#pragma once

#include <cstdint>

class Barometer
{
private:
    static constexpr const char * sPressurePath = "/sys/bus/iio/devices/ms5611/in_voltage0_raw";
    static constexpr const char * sTemperaturePath = "/sys/bus/iio/devices/ms5611/in_temp0_raw";

public:
    Barometer();

    double GetPressure();
    double GetTemperature();
};
