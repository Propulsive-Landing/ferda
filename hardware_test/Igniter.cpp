#include "Igniter.hpp"

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    if (ignite == Igniter::IgnitionSpecifier::LAUNCH)
        cout<<"Ignite"<<"\n"
    else if (ignite == Igniter::IgnitionSpecifier::ABORT)
        cout<<"Abort"<<"\n"

    return;
}
}
