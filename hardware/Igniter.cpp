#include "Igniter.hpp"
#include <pigpio.h>

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    gpioWrite(5, 0);
}

void Igniter::DisableIgnite(Igniter::IgnitionSpecifier ignite)
{
    gpioWrite(5, 1);
}
