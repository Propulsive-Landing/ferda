#include "Barometer.hpp"

Barometer::Barometer(uint8_t byAddress) : m_byAddress(byAddress)
{
}

double Barometer::GetPressure()
{
    return 0;
}
