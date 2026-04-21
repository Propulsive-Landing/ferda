#include "TVC.hpp"

#include <Eigen/Dense>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <MissionConstants.hpp>
#include <iostream>
#include <string>
#include "Telemetry.hpp"
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
                                  double& length_x, double& length_y)
{
    // Convert gimbal angles to linear actuator lengths using mechanical geometry.
    // Origin: TVC u-joint
    // Commanded angles define rotation matrix R (rotation in XY plane from engine frame)
    // Actuator lengths = ||vehicle_mount_point - R * engine_mount_point||

    // Get mounting point coordinates from mission constants
    Eigen::Vector3d vehicle_mount_0 = MissionConstants::kTvcVehicleMountPoint0;
    Eigen::Vector3d vehicle_mount_1 = MissionConstants::kTvcVehicleMountPoint1;
    Eigen::Vector3d engine_mount_0 = MissionConstants::kTvcEngineMountPoint0;
    Eigen::Vector3d engine_mount_1 = MissionConstants::kTvcEngineMountPoint1;

    // ============================================================================
    // Build rotation matrix from gimbal angles (axis-angle representation)
    // ============================================================================
    // The gimbal angles (angle_x, angle_y) define a rotation vector in the XY plane.
    // Use Rodrigues' formula: the rotation axis is the vector (angle_x, angle_y, 0),
    // normalized, and the rotation magnitude is the vector's norm.
    // This avoids gimbal lock and rotation order ambiguity.

    Eigen::Vector3d rotation_vector(angle_x_rad, angle_y_rad, 0.0);
    double rotation_angle = rotation_vector.norm();
    
    Eigen::Matrix3d R;
    if (rotation_angle > 1e-10)
    {
        // Non-zero rotation: use axis-angle (Rodrigues' formula via AngleAxisd)
        Eigen::Vector3d rotation_axis = rotation_vector.normalized();
        R = Eigen::AngleAxisd(rotation_angle, rotation_axis).toRotationMatrix();
    }
    else
    {
        // Near-zero rotation: identity matrix
        R = Eigen::Matrix3d::Identity();
    }

    // ============================================================================
    // Compute actuator lengths from rotated engine mounting points
    // ============================================================================
    Eigen::Vector3d engine_mount_0_rotated = R * engine_mount_0;
    Eigen::Vector3d engine_mount_1_rotated = R * engine_mount_1;

    Eigen::Vector3d vector_0 = vehicle_mount_0 - engine_mount_0_rotated;
    Eigen::Vector3d vector_1 = vehicle_mount_1 - engine_mount_1_rotated;

    // Convert from absolute length to extension length by subtracting the minimum length (when fully retracted)
    length_x = vector_0.norm() - MissionConstants::kTvcZeroExtensionInches;  
    length_y = vector_1.norm() - MissionConstants::kTvcZeroExtensionInches;

    // Clamp to valid actuator range
    length_x = std::clamp(length_x, MissionConstants::kTvcMinLengthInches, MissionConstants::kTvcMaxLengthInches);
    length_y = std::clamp(length_y, MissionConstants::kTvcMinLengthInches, MissionConstants::kTvcMaxLengthInches);
}

void TVC::ProportionalPositionControl(int actuator_index, double dt_seconds)
{
    // actuator_index: 0 = X axis, 1 = Y axis
    double current_length = current_actuator_lengths(actuator_index);
    double desired_length = desired_actuator_lengths(actuator_index);

    // Position error [inches]
    double error = desired_length - current_length;

    if (dt_seconds <= 1e-6)
    {
        dt_seconds = 1e-3;
    }

    integral_error_inch_seconds(actuator_index) += error * dt_seconds;
    integral_error_inch_seconds(actuator_index) = std::clamp(
        integral_error_inch_seconds(actuator_index),
        -MissionConstants::kTvcIntegralWindupLimitInchSeconds,
        MissionConstants::kTvcIntegralWindupLimitInchSeconds);

    const double derivative_error_inches_per_second =
        (error - previous_error_inches(actuator_index)) / dt_seconds;
    previous_error_inches(actuator_index) = error;

    // PID output is target actuator velocity [in/s].
    double velocity_command_inches_per_second =
        MissionConstants::kTvcPositionKpPerSecond * error
        + MissionConstants::kTvcPositionKiPerSecondSquared * integral_error_inch_seconds(actuator_index)
        + MissionConstants::kTvcPositionKdUnitless * derivative_error_inches_per_second;

    velocity_command_inches_per_second = std::clamp(
        velocity_command_inches_per_second,
        -MissionConstants::kTvcMaxCommandedVelocityInchesPerSecond,
        MissionConstants::kTvcMaxCommandedVelocityInchesPerSecond);

    // Convert commanded velocity to motor direction and speed.
    int direction = 0;  // 0 = stop, 1 = extend, -1 = retract
    if (velocity_command_inches_per_second > MissionConstants::kTvcVelocityDeadbandInchesPerSecond)
    {
        direction = 1;  // Extend
    }
    else if (velocity_command_inches_per_second < -MissionConstants::kTvcVelocityDeadbandInchesPerSecond)
    {
        direction = -1; // Retract
    }

    double normalized_speed_command = 0.0;
    if (MissionConstants::kTvcMaxCommandedVelocityInchesPerSecond > 1e-6)
    {
        normalized_speed_command = std::abs(velocity_command_inches_per_second)
            / MissionConstants::kTvcMaxCommandedVelocityInchesPerSecond;
    }
    normalized_speed_command = std::clamp(normalized_speed_command, 0.0, 1.0);

    int speed_cmd = static_cast<int>(normalized_speed_command * MissionConstants::kTvcMaxMotorSpeed);
    speed_cmd = std::clamp(speed_cmd, 0, MissionConstants::kTvcMaxMotorSpeed);

    if (actuator_index == 0)
    {
        last_speed_command_x = speed_cmd;
    }
    else
    {
        last_speed_command_y = speed_cmd;
    }

    // Send velocity command to motor
    driveActuator(actuator_index, direction, speed_cmd);
}

void TVC::SetTVCX(double angle_rad)
{
    // Apply compile-time calibration trim before clamping.
    const double corrected_angle = angle_rad + MissionConstants::kTvcXInputCenterAngleRad;
    stored_angle_x_rad = std::clamp(corrected_angle, -10.0 * MissionConstants::kDeg2Rad, 10.0 * MissionConstants::kDeg2Rad);
    last_command_time = std::chrono::steady_clock::now();
    has_recent_command = true;
}

void TVC::SetTVCY(double angle_rad)
{
    // Apply compile-time calibration trim before clamping.
    const double corrected_angle = angle_rad + MissionConstants::kTvcYInputCenterAngleRad;
    stored_angle_y_rad = std::clamp(corrected_angle, -10.0 * MissionConstants::kDeg2Rad, 10.0 * MissionConstants::kDeg2Rad);
    last_command_time = std::chrono::steady_clock::now();
    has_recent_command = true;
}

void TVC::UpdateActuatorPositions()
{
    const auto now = std::chrono::steady_clock::now();
    if (!has_recent_command)
    {
        Stop();
        return;
    }

    const double command_age_seconds = std::chrono::duration<double>(now - last_command_time).count();
    if (command_age_seconds > MissionConstants::kTvcCommandTimeoutSeconds)
    {
        Stop();
        return;
    }

    // Compute desired actuator lengths from both stored angles (eliminates coupling ambiguity)
    double length_x, length_y;
    AnglesToActuatorLengths(stored_angle_x_rad, stored_angle_y_rad, length_x, length_y);
    desired_actuator_lengths(0) = length_x;
    desired_actuator_lengths(1) = length_y;

    // Read current actuator positions from sensors
    current_actuator_lengths(0) = readPositionInches(0);
    current_actuator_lengths(1) = readPositionInches(1); 

    double dt_seconds = 0.005;
    if (has_control_timestamp)
    {
        dt_seconds = std::chrono::duration<double>(now - last_control_time).count();
    }
    last_control_time = now;
    has_control_timestamp = true;

    // Run proportional position control for each actuator
    ProportionalPositionControl(0, dt_seconds);
    ProportionalPositionControl(1, dt_seconds);

    Telemetry::GetInstance().LogActuatorFrame(stored_angle_x_rad,
                                              stored_angle_y_rad,
                                              desired_actuator_lengths(0),
                                              desired_actuator_lengths(1),
                                              current_actuator_lengths(0),
                                              current_actuator_lengths(1),
                                              last_speed_command_x,
                                              last_speed_command_y);
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
