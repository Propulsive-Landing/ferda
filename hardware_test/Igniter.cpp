#include "Igniter.hpp"
#include <iostream>

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    std::cout << "Launch Motor Ignition Started" << "\n";
    return;
}

void Igniter::DisableIgnite(Igniter::IgnitionSpecifier ignite)
{
    std::cout << "Disable Launch Igniter" << "\n";
}