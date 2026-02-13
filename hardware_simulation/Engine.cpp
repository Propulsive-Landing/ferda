// hardware_simulation/Engine.cpp
#include "Engine.hpp"
#include "UDPClient.hpp"

void Engine::SetThrust(double thrust_N) {
    UDPClient::GetInstance().SetThrust(thrust_N);
}