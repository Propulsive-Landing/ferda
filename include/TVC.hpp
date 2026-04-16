#pragma once

#include <Eigen/Dense>
#include "MissionConstants.hpp"

#ifdef NDEBUG
#include <PiPCA9685/PCA9685.h>
#endif

class TVC
{
private:
    // Linear actuator control
    Eigen::Vector2d desired_actuator_lengths;  // [length_x, length_y] in inches
    Eigen::Vector2d current_actuator_lengths;  // [length_x, length_y] in inches
    double stored_angle_x_rad = 0.0;           // Stored gimbal angle X [rad]
    double stored_angle_y_rad = 0.0;           // Stored gimbal angle Y [rad]

    // PiPCA9685::PCA9685 dev;

    // Helper functions
    void AnglesToActuatorLengths(double angle_x_rad, double angle_y_rad,
                                 double& length_x, double& length_y);
    void ProportionalPositionControl(int actuator_index);

public:
    TVC() = default;
    void SetTVCX(double angle_rad);
    void SetTVCY(double angle_rad);
    void UpdateActuatorPositions();
};