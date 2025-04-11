#include "Igniter.hpp"
#include <pigpio.h>

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
    {
        gpioWrite(21, 0);
    }
    else if (ignite == Igniter::IgnitionSpecifier::LAND)
    {
        gpioWrite(22, 0);
    }
}

void Igniter::DisableIgnite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
    {
        gpioWrite(21, 1);
    }
    else if (ignite == Igniter::IgnitionSpecifier::LAND)
    {
        gpioWrite(22, 1);
    }
}
