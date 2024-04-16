#include "Igniter.hpp"
#include <iostream>

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
        std::cout<<"Launch Motor Ignition Started"<<"\n";
    else if (ignite == Igniter::IgnitionSpecifier::LAND)
        std::cout<<"Land"<<"\n";

    return;
}


void Igniter::DisableIgnite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
    {
        std::cout<<"Disable Launch Igniter"<<"\n";
    }
    else if (ignite == Igniter::IgnitionSpecifier::LAND)
    {
        // gpioWrite(17, 1);
    }
}