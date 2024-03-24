#include "Igniter.hpp"
#include <iostream>

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
        std::cout<<"Ignite"<<"\n";
    else if (ignite == Igniter::IgnitionSpecifier::LAND)
        std::cout<<"Ignite"<<"\n";

    return;
}
