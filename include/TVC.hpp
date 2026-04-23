#pragma once

#include <Eigen/Dense>
#include <chrono>
#include "MissionConstants.hpp"

class TVC
{
private:
    // Linear actuator control
    Eigen::Vector2d desired_actuator_lengths; // [length_x, length_y] in inches
    Eigen::Vector2d current_actuator_lengths; // [length_x, length_y] in inches
    double stored_angle_x_rad = 0.0;          // Stored gimbal angle X [rad]
    double stored_angle_y_rad = 0.0;          // Stored gimbal angle Y [rad]
    std::chrono::steady_clock::time_point last_command_time;
    std::chrono::steady_clock::time_point last_control_time;
    bool has_recent_command = false;
    bool has_control_timestamp = false;
    int last_speed_command_x = 0;
    int last_speed_command_y = 0;
    Eigen::Vector2d integral_error_inch_seconds = Eigen::Vector2d::Zero();
    Eigen::Vector2d previous_error_inches = Eigen::Vector2d::Zero();

    // Helper functions
    void AnglesToActuatorLengths(double angle_x_rad, double angle_y_rad,
                                 double &length_x, double &length_y);
    void ProportionalPositionControl(int actuator_index, double dt_seconds);

public:
    TVC();
    void SetTVCX(double angle_rad);
    void SetTVCY(double angle_rad);
    void UpdateActuatorPositions();
    void Stop();
    Eigen::Vector2d GetActuatorSetpointErrorInches() const
    {
        return desired_actuator_lengths - current_actuator_lengths;
    }
};