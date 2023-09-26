#pragma once

#include <cstdint>

class Barometer
{
private:
    uint8_t m_byAddress;

public:
    Barometer(uint8_t byAddress = 0x77);

    double GetPressure();
};
