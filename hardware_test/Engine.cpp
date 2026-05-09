// Update Engine class for hardware implementation

// hardware_test/Engine.cpp
#include "Engine.hpp"

void Engine::SetThrust(double thrust_N, const Eigen::Vector3d &position_e, const Eigen::Vector3d &velocity_e) {
    (void)position_e;
    (void)velocity_e;
    return;
}