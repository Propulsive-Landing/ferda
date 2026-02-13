#pragma once

#include <Eigen/Dense>

#include "MissionConstants.hpp"

class Engine
{
    public:
        Engine() = default;
        void SetThrust(double thrust_N); 
};