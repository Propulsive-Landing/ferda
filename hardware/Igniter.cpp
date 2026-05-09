#include <wiringPi.h>
#include "Igniter.hpp"
#include "MissionConstants.hpp"

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    // digitalWrite(MissionConstants::kIgnitionPin, 0);
}

void Igniter::DisableIgnite(Igniter::IgnitionSpecifier ignite)
{
    // digitalWrite(MissionConstants::kIgnitionPin, 1);
}
