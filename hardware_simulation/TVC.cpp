// hardware_simulation/TVC.cpp
#include "TVC.hpp"
#include "UDPClient.hpp"

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

void TVC::SetTVCX(double angle_rad) {
    stored_angle_x_rad = angle_rad;
    has_recent_command = true;
    last_command_time = std::chrono::steady_clock::now();
    UDPClient::GetInstance().SetTVCX(angle_rad);
}

void TVC::SetTVCY(double angle_rad) {
    stored_angle_y_rad = angle_rad;
    has_recent_command = true;
    last_command_time = std::chrono::steady_clock::now();
    UDPClient::GetInstance().SetTVCY(angle_rad);
}

void TVC::UpdateActuatorPositions()
{
    // In simulation, SetTVCX/SetTVCY immediately forward commands via UDP.
}

void TVC::Stop()
{
    has_recent_command = false;
    has_control_timestamp = false;
    stored_angle_x_rad = 0.0;
    stored_angle_y_rad = 0.0;
    UDPClient::GetInstance().SetTVCX(0.0);
    UDPClient::GetInstance().SetTVCY(0.0);
}