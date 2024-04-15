#include "Igniter.hpp"
#include <pigpio.h>


void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
    {
        gpioWrite(6, 0);
    }
    else if (ignite == Igniter::IgnitionSpecifier::LAND)
    {
        // gpioWrite(17, 1);
    }
}

void Igniter::DisableIgnite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
    {
        gpioWrite(6, 1);
    }
    else if (ignite == Igniter::IgnitionSpecifier::LAND)
    {
        // gpioWrite(17, 1);
    }
}
