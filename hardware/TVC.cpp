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

void TVC::ProportionalPositionControl(int actuator_index)
{
    // actuator_index: 0 = X axis, 1 = Y axis
    double current_length = current_actuator_lengths(actuator_index);
    double desired_length = desired_actuator_lengths(actuator_index);

    // Position error [inches]
    double error = desired_length - current_length;

    // Proportional velocity command: larger error -> larger command
    // Normalized to [-1, 1] range where 1.0 = max motor speed
    double velocity_command = MissionConstants::kTvcPositionControlGain * error / MissionConstants::kTvcMaxMotorSpeed;
    velocity_command = std::clamp(velocity_command, -1.0, 1.0);

    // Convert normalized velocity to motor direction and speed
    int direction = 0;  // 0 = stop, 1 = extend, -1 = retract
    if (velocity_command > 0.01)      // Deadband to avoid chatter
    {
        direction = 1;  // Extend
    }
    else if (velocity_command < -0.01)
    {
        direction = -1; // Retract
    }

    int speed_cmd = static_cast<int>(std::abs(velocity_command) * MissionConstants::kTvcMaxMotorSpeed);
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
    // Clamp and store angle for use in UpdateActuatorPositions()
    stored_angle_x_rad = std::clamp(angle_rad, -10.0 * MissionConstants::kDeg2Rad, 10.0 * MissionConstants::kDeg2Rad);
}

void TVC::SetTVCY(double angle_rad)
{
    // Clamp and store angle for use in UpdateActuatorPositions()
    stored_angle_y_rad = std::clamp(angle_rad, -10.0 * MissionConstants::kDeg2Rad, 10.0 * MissionConstants::kDeg2Rad);
}

void TVC::UpdateActuatorPositions()
{
    // Compute desired actuator lengths from both stored angles (eliminates coupling ambiguity)
    double length_x, length_y;
    AnglesToActuatorLengths(stored_angle_x_rad, stored_angle_y_rad, length_x, length_y);
    desired_actuator_lengths(0) = length_x;
    desired_actuator_lengths(1) = length_y;

    // Read current actuator positions from sensors
    current_actuator_lengths(0) = readPositionInches(0);
    current_actuator_lengths(1) = readPositionInches(1); 

    // Run proportional position control for each actuator
    ProportionalPositionControl(0);
    ProportionalPositionControl(1);

    Telemetry::GetInstance().LogActuatorFrame(stored_angle_x_rad,
                                              stored_angle_y_rad,
                                              desired_actuator_lengths(0),
                                              desired_actuator_lengths(1),
                                              current_actuator_lengths(0),
                                              current_actuator_lengths(1),
                                              last_speed_command_x,
                                              last_speed_command_y);
}
