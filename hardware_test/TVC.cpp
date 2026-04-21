#include "TVC.hpp"

#include <iostream>
#include <string>
#include "LinActMotorPositionControl.hpp"

TVC::TVC()
    : desired_actuator_lengths(Eigen::Vector2d::Zero()),
      current_actuator_lengths(Eigen::Vector2d::Zero()),
      last_command_time(std::chrono::steady_clock::now()),
      last_control_time(std::chrono::steady_clock::now()),
      has_recent_command(false),
      has_control_timestamp(false),
      last_speed_command_x(0),
      last_speed_command_y(0)
{
}

void TVC::AnglesToActuatorLengths(double angle_x_rad, double angle_y_rad,
                                  double &length_x, double &length_y)
{
}

void TVC::ProportionalPositionControl(int actuator_index, double dt_seconds)
{
}

void TVC::SetTVCX(double dAngle)
{
    std::cout << "Wrote angle to X: " + std::to_string(dAngle) + " PW: " + std::to_string(0) + "\n";
}

void TVC::SetTVCY(double dAngle)
{

    std::cout << "Wrote angle to Y: " + std::to_string(dAngle) + " PW: " + std::to_string(0) + "\n";
}

void TVC::UpdateActuatorPositions()
{
}

void TVC::Stop()
{
    has_recent_command = false;
    has_control_timestamp = false;
    stored_angle_x_rad = 0.0;
    stored_angle_y_rad = 0.0;
    desired_actuator_lengths = current_actuator_lengths;
    last_speed_command_x = 0;
    last_speed_command_y = 0;
    integral_error_inch_seconds = Eigen::Vector2d::Zero();
    previous_error_inches = Eigen::Vector2d::Zero();

    driveActuator(0, 0, 0);
    driveActuator(1, 0, 0);
}