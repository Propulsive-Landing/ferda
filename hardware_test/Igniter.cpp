#include "Igniter.hpp"
#include <iostream>

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
        std::cout << "Launch Motor Ignition Started" << "\n";

    else if (ignite == Igniter::IgnitionSpecifier::LAND)
        std::cout << "Land Motor Ignition Started" << "\n";

    return;
}

void Igniter::DisableIgnite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
    {
        std::cout << "Disable Launch Igniter" << "\n";
    }
    else if (ignite == Igniter::IgnitionSpecifier::LAND)
    {
        std::cout << "Disable Land Igniter" << "\n";
    }
}