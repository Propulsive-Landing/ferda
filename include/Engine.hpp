#pragma once

#include <Eigen/Dense>

#include "MissionConstants.hpp"

class Engine
{
    public:
        Engine() = default;
    void SetThrust(double thrust_N, const Eigen::Vector3d &position_e, const Eigen::Vector3d &velocity_e);
};