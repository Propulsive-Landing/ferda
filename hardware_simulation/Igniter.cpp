// hardware_simulation/Igniter.cpp
#include "Igniter.hpp"
#include "UDPClient.hpp"

void Igniter::Ignite(IgnitionSpecifier ignite) {
    UDPClient::GetInstance().Ignite(ignite == IgnitionSpecifier::LAUNCH);
}

void Igniter::DisableIgnite(IgnitionSpecifier ignite) {
    // No action needed for simulation
}