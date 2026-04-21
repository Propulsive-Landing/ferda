#include <Eigen/Dense>

#include "Controller.hpp"
#include "Telemetry.hpp"
#include "MissionConstants.hpp"
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

Controller::Controller(TVC &inputTvc, Engine &inputEngine) : x_control(Eigen::Matrix<double, 6, 1>::Zero()), tvc(inputTvc), engine(inputEngine) {}

void Controller::Start(double current_time)
{
    // Initialize variables
    error_integral = Eigen::Vector2d::Zero();
    height_error_integral = 0.0;
    translation_error_integral = Eigen::Vector2d::Zero();
    setpoint_angles = Eigen::Vector2d::Zero();
    setpoint_angles_prev = Eigen::Vector2d::Zero();
}

void Controller::UpdateTestTVC(double testTime)
{

    double angleA = 0 * sin(testTime) * MissionConstants::kMaximumTvcAngle; // Rad
    double angleB = 0 * cos(testTime) * MissionConstants::kMaximumTvcAngle; // Rad

    input(0) = angleA;
    input(1) = angleB;
    // [TOD] Move to hardware tvc_angles = TvcMath(input);

    tvc.SetTVCX(input(0));
    tvc.SetTVCY(input(1));
    tvc.UpdateActuatorPositions();
}

void Controller::UpdateLaunch(Navigation &navigation, double current_time)
{
    // Use the TVC to stabilize the rocket for landing

    TranslationControl(navigation);
    AttitudeControl(navigation);
    HeightControl(navigation);
}

void Controller::AttitudeControl(Navigation &navigation)
{
    // Create a matrix to store the returned stateEstimate from getNavigation() and store the yaw value into a varible
    Eigen::Matrix<double, 16, 1> stateEstimate = navigation.GetNavigation();
    Eigen::Vector3d angularVelocity = navigation.GetAngularVelocity();

    q_current = Eigen::Quaterniond(
        stateEstimate(6),   // w
        stateEstimate(7),   // x
        stateEstimate(8),   // y
        stateEstimate(9)    // z
    );
    Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();
    q_error = q_current * q_ref.inverse();

    Eigen::Vector2d theta = 2 * Eigen::Vector2d(q_error.x(), q_error.y());
    Eigen::Vector2d theta_error = setpoint_angles - theta;
    error_integral = error_integral + theta_error * loopTime;

    // Approximate the derivative of theta_error using finite differences
    Eigen::Vector2d setpoint_angles_dot = (setpoint_angles - setpoint_angles_prev) / loopTime;
    
    // Calculate velocity error: derivative of setpoint angles minus current angular velocity
    Eigen::Vector2d velocity_error = setpoint_angles_dot - angularVelocity.segment(0, 2);

    // Update previous setpoint_angles for next iteration
    setpoint_angles_prev = setpoint_angles;

    x_control.segment(0, 2) =  error_integral;                 // Body frame x and y velocities
    x_control.segment(2, 2) =  theta_error;                    // Roll and pitch error relative to setpoint
    x_control.segment(4, 2) =  velocity_error;                 // Velocity error term

    CalculateInput(navigation);
}

// shut down rocket functions
void Controller::UpdateSafe()
{
    // TODO. Center TVC, turn off reaction wheel, etc.
}

void Controller::CalculateInput(Navigation &navigation)
{
    // This calculates u = -Kx

    const Eigen::Vector3d estimatedMoiBody = navigation.GetEstimatedMomentOfInertiaBodyKgm2();
    const Eigen::Vector3d estimatedComBody = navigation.GetEstimatedCenterOfMassBodyM();
    const double lateralMoi = 0.5 * (estimatedMoiBody(0) + estimatedMoiBody(1));
    const double thrustForScaling = std::max(std::abs(current_thrust_command_N), MissionConstants::kEngineMinThrust);
    const double momentArm = std::abs(estimatedComBody(2) - MissionConstants::kEngineThrustLocationBodyM(2));
    const double controlScale = lateralMoi / (thrustForScaling * momentArm);

    input = -controlScale * angle_controller_gains * x_control;

    if (input.norm() > MissionConstants::kMaximumTvcAngle)
    {
        input = input * MissionConstants::kMaximumTvcAngle / input.norm();
    }

    // Figures out what angle we need to move the servos and then set them
    // [TODO] Move to hardware tvc_angles = TvcMath(input);
    tvc.SetTVCX(input(0));
    tvc.SetTVCY(input(1));
    tvc.UpdateActuatorPositions();
}

void Controller::TranslationControl(Navigation &navigation)
{
Eigen::Matrix<double, 16, 1> x = navigation.GetNavigation();

    // States (earth frame)
    double px = x(0);   // x position [m]
    double py = x(1);   // y position [m]
    double vx = x(3);   // x velocity [m/s]
    double vy = x(4);   // y velocity [m/s]

    // Errors
    double e_x  = refPositionX - px;
    double e_y  = refPositionY - py;
    double e_vx = refVelocityX - vx;
    double e_vy = refVelocityY - vy;

    // Integral update
    translation_error_integral(0) += e_x * loopTime;
    translation_error_integral(1) += e_y * loopTime;

    // Build state vector: [x_int_err, y_int_err, e_x, e_y, e_vx, e_vy]
    Eigen::Matrix<double, 6, 1> x_translation;
    x_translation << translation_error_integral(0),
                     translation_error_integral(1),
                     e_x, e_y,
                     e_vx, e_vy;

    // Compute setpoint angles: [roll_setpoint, pitch_setpoint]
    double mass = navigation.GetEstimatedMassKg();
    double thrust = current_thrust_command_N;
    if (std::abs(thrust) < 1e-6)
    {
        thrust = MissionConstants::kEngineMinThrust;
    }
    setpoint_angles = translation_controller_gains * x_translation * (mass / thrust);
}

void Controller::HeightControl(Navigation& navigation)
{
    const double g = MissionConstants::kGravity; // m/s^2

    Eigen::Matrix<double, 16, 1> x = navigation.GetNavigation();

    // States
    double z     = x(2);   // position [m]
    double zdot  = x(5);   // velocity [m/s]

    // References
    double z_ref    = refPositionZ;
    double zdot_ref = refVelocityZ;

    // Errors
    double e_z    = z_ref - z;
    double e_zdot = zdot_ref - zdot;

    // Integral update
    height_error_integral += e_z * loopTime;

    // Acceleration command
    Eigen::Vector3d height_control_vector = Eigen::Vector3d(height_error_integral, e_z, e_zdot);
    double zddot_cmd = height_controller_gains * height_control_vector + refAccelerationZ;

    // Use navigation's current vehicle mass estimate
    double mass = navigation.GetEstimatedMassKg(); // kg

    // Convert to force and clamp to engine throttle limits
    double thrust_cmd = (mass * (zddot_cmd + g));
    if (thrust_cmd < MissionConstants::kEngineMinThrust)
    {
        thrust_cmd = MissionConstants::kEngineMinThrust;
    }
    else if (thrust_cmd > MissionConstants::kEngineMaxThrust)
    {
        thrust_cmd = MissionConstants::kEngineMaxThrust;
    }

    current_thrust_command_N = thrust_cmd;
    engine.SetThrust(thrust_cmd, x.segment<3>(0), x.segment<3>(3)); // Newtons + navigation state
}

void Controller::Center()
{
    // Center the tvc
    input(0) = 0;
    input(1) = 0;
    // [TODO] Move to hardware tvc_angles = TvcMath(input);
    tvc.SetTVCX(input(0));
    tvc.SetTVCY(input(1));
    tvc.UpdateActuatorPositions();
}

void Controller::ImportHeightParameters(std::string file_name)
{
    // Imports the kmatrix file into controller_gains
    
    char separator = ',';
    std::string row, item;
    std::ifstream in(file_name);

    if (!in.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    // Get the controller values of the k-matrix
    int rows_read = 0;
    while (rows_read < 1 && std::getline(in, row)) {
        row.erase(std::remove_if(row.begin(), row.end(), ::isspace), row.end());
        if (row.empty()) continue;
        std::stringstream controllerValueStringStream(row);
        for (int j = 0; j < 3; j++) {
            if (!std::getline(controllerValueStringStream, item, separator)) {
                throw std::runtime_error("Not enough columns in row for height_controller_gains");
            }
            try {
                height_controller_gains(rows_read, j) = std::stod(item);
            } catch (const std::invalid_argument& e) {
                throw std::runtime_error("Invalid number in CSV for height_controller_gains: '" + item + "'");
            }
        }
        rows_read++;
    }
    if (rows_read < 1) {
        throw std::runtime_error("Not enough rows in height_controller_gains CSV");
    }

    in.close();
}

void Controller::ImportTranslationParameters(std::string file_name)
{
    // Imports the kmatrix file into controller_gains

    char separator = ',';
    std::string row, item;
    std::ifstream in(file_name);

    if (!in.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    // Get the controller values of the k-matrix
    int rows_read = 0;
    while (rows_read < 2 && std::getline(in, row)) {
        row.erase(std::remove_if(row.begin(), row.end(), ::isspace), row.end());
        if (row.empty()) continue;
        std::stringstream controllerValueStringStream(row);
        for (int j = 0; j < 6; j++) {
            if (!std::getline(controllerValueStringStream, item, separator)) {
                throw std::runtime_error("Not enough columns in row for translation_controller_gains");
            }
            try {
                translation_controller_gains(rows_read, j) = std::stod(item);
            } catch (const std::invalid_argument& e) {
                throw std::runtime_error("Invalid number in CSV for translation_controller_gains: '" + item + "'");
            }
        }
        rows_read++;
    }
    if (rows_read < 2) {
        throw std::runtime_error("Not enough rows in translation_controller_gains CSV");
    }

    in.close();
}

void Controller::ImportAngleParameters(std::string file_name)
{
    // Imports the kmatrix file into controller_gains

    char separator = ',';
    std::string row, item;
    std::ifstream in(file_name);

    if (!in.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    // Get the controller values of the k-matrix
    int rows_read = 0;
    while (rows_read < 2 && std::getline(in, row)) {
        row.erase(std::remove_if(row.begin(), row.end(), ::isspace), row.end());
        if (row.empty()) continue;
        std::stringstream controllerValueStringStream(row);
        for (int j = 0; j < 6; j++) {
            if (!std::getline(controllerValueStringStream, item, separator)) {
                throw std::runtime_error("Not enough columns in row for angle_controller_gains");
            }
            try {
                angle_controller_gains(rows_read, j) = std::stod(item);
            } catch (const std::invalid_argument& e) {
                throw std::runtime_error("Invalid number in CSV for angle_controller_gains: '" + item + "'");
            }
        }
        rows_read++;
    }
    if (rows_read < 2) {
        throw std::runtime_error("Not enough rows in angle_controller_gains CSV");
    }

    in.close();
}

Eigen::Matrix<double, 2, 1> Controller::GetCurrentTVCCommand()
{
    return input;
}

double Controller::GetCurrentThrustCommand()
{
    return current_thrust_command_N;
}