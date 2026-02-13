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
    next_tvc_time = current_time;
    error_integral = Eigen::Vector3d::Zero();
}

void Controller::UpdateTestTVC(double testTime)
{

    double angleA = sin(testTime) * MissionConstants::kMaximumTvcAngle; // Rad
    double angleB = cos(testTime) * MissionConstants::kMaximumTvcAngle; // Rad

    input(0) = angleA;
    input(1) = angleB;
    // [TOD] Move to hardware tvc_angles = TvcMath(input);

    tvc.SetTVCX(input(0));
    tvc.SetTVCY(input(1));
}

void Controller::UpdateLaunch(Navigation &navigation, double current_time)
{
    // Use the TVC to stabilize the rocket for landing

    AttitudeControl(navigation);
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
    q_error = q_ref * q_current.inverse();

    Eigen::Vector3d error_vec = Eigen::Vector3d(2*q_error.x(), 2*q_error.y(), 2*q_error.z());
    error_integral = error_integral + error_vec * loopTime;

    x_control.segment(0, 2) =  angularVelocity.segment(0, 2); // Body frame x and y velocities
    x_control.segment(2, 2) =  error_vec.segment(0, 2); // Body frame roll and pitch angles
    x_control.segment(4, 2) =  error_integral.segment(0, 2);

    CalculateInput();
    next_tvc_time += MissionConstants::TVCPeriod;
}

// shut down rocket functions
void Controller::UpdateSafe()
{
    // TODO. Center TVC, turn off reaction wheel, etc.
}

void Controller::CalculateInput()
{
    // This calculates u = -Kx

    input = angle_controller_gains * x_control;
    if (input.norm() > MissionConstants::kMaximumTvcAngle)
    {
        input = input * MissionConstants::kMaximumTvcAngle / input.norm();
    }

    // Figures out what angle we need to move the servos and then set them
    // [TODO] Move to hardware tvc_angles = TvcMath(input);
    tvc.SetTVCX(input(0));
    tvc.SetTVCY(input(1));
}

void Controller::HeightControl(Navigation& navigation)
{
    constexpr double g = 9.80665;

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
    
    double zddot_cmd = height_controller_gains * height_control_vector;

    //Placeholder mass. TODO: estimate mass over time
    double mass = 100; // kg

    // Convert to force
    double thrust_cmd = mass * (zddot_cmd + g);

    engine.SetThrust(thrust_cmd); // Newtons
}

void Controller::Center()
{
    // Center the tvc
    input(0) = 0;
    input(1) = 0;
    // [TODO] Move to hardware tvc_angles = TvcMath(input);
    tvc.SetTVCX(input(0));
    tvc.SetTVCY(input(1));
}

void Controller::ImportHeightParameters(std::string file_name)
{
    // Imports the kmatrix file into controller_gains
    
    char separator = ',';
    std::string row, item;
    std::ifstream in(file_name);
    std::getline(in, row);

    if (!in.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    // Get the controller values of the k-matrix
    for (int i = 0; i < 2; i++)
    {
        std::getline(in, row);
        std::stringstream controllerValueStringStream(row);
        for (int j = 0; j < 6; j++)
        {
            std::getline(controllerValueStringStream, item, separator);
            height_controller_gains(i, j) = stod(item);
        }
    }

    in.close();
}

void Controller::ImportTranslationParameters(std::string file_name)
{
    // Imports the kmatrix file into controller_gains

    char separator = ',';
    std::string row, item;
    std::ifstream in(file_name);
    std::getline(in, row);

    if (!in.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    // Get the controller values of the k-matrix
    for (int i = 0; i < 1; i++)
    {
        std::getline(in, row);
        std::stringstream controllerValueStringStream(row);
        for (int j = 0; j < 3; j++)
        {
            std::getline(controllerValueStringStream, item, separator);
            translation_controller_gains(i, j) = stod(item);
        }
    }

    in.close();
}

void Controller::ImportAngleParameters(std::string file_name)
{
    // Imports the kmatrix file into controller_gains

    char separator = ',';
    std::string row, item;
    std::ifstream in(file_name);
    std::getline(in, row);

    if (!in.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    // Get the controller values of the k-matrix
    for (int i = 0; i < 2; i++)
    {
        std::getline(in, row);
        std::stringstream controllerValueStringStream(row);
        for (int j = 0; j < 6; j++)
        {
            std::getline(controllerValueStringStream, item, separator);
            angle_controller_gains(i, j) = stod(item);
        }
    }

    in.close();
}

int Controller::GetCurrentIterationIndex()
{
    return current_iteration_index;
}

Eigen::Matrix<double, 2, 1> Controller::GetCurrentTVCCommand()
{
    return input;
}