// hardware_simulation/Engine.cpp
#include "Engine.hpp"
#include "UDPClient.hpp"

void Engine::SetThrust(double thrust_N, const Eigen::Vector3d &position_e, const Eigen::Vector3d &velocity_e) {
    UDPClient::GetInstance().SetNavigationState(position_e, velocity_e);
    UDPClient::GetInstance().SetThrust(thrust_N);
}