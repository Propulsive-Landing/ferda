#include "Igniter.hpp"

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
        gpioWrite(4, 1);
    else if (ignite == Igniter::IgnitionSpecifier::ABORT)
        gpioWrite(17, 1);

    return;
}
