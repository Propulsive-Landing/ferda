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

Controller::Controller(TVC &inputTvc) : x_control(Eigen::Matrix<double, 8, 1>::Zero()), controller_gain_times(10, 0), tvc(inputTvc) {}

void Controller::Start(double current_time)
{
    // Initialize variables
    next_tvc_time = current_time;
    ResetKIteration(current_time); // Sets iteration start time
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

    Center();
}

void Controller::stabilizeAtCenter(Navigation &navigation, double current_time)
{
    // IN X_CONRTOL:: FIRST 2 ARE X AND Y VELOCITIES, NEXT 2 ARE EULER INTEGRALS, NEXT 2 ARE ROLL AND PITCH, AND NEXT 2 ARE ROLl and pitch values

    // Calculate desired control inputs for launch and actuate all control surfaces accordingly

    // Create a variable to determine the max amount of Euler Entries;
    unsigned int maxEulerEntries = MissionConstants::kControlIntegralPeriod / loopTime;

    // Create a matrix to store the returned stateEstimate from getNavigation() and store the yaw value into a varible
    //TODO: This is incorrect
    Eigen::Matrix<double, 16, 1> stateEstimate = navigation.GetNavigation();

    // Extract yaw
    double yaw = stateEstimate(8);

    // Calculate the rotation matrix to translate the earth frame to the body frame
    Eigen::Matrix2d rotation;
    rotation << cos(-yaw), -sin(-yaw), sin(-yaw), cos(-yaw);

    // Populate x_control so that the first 2 (0,1) entries are the stateEstimates' x and y velocities, its 4-5 entries are stateEstimates' roll and pitch values
    // and it's last 2 entries are stateEstimates' roll and pitch angular velocities.

    x_control.segment(0, 2) = rotation * stateEstimate.segment(3, 2);

    // Control velocity by offseting the angle set point based on the current body frame velocity

    // phi_adjusted = phi - v_y*weights_control_velocity
    stateEstimate[6] = stateEstimate[6] - x_control(1) * MissionConstants::weights_control_velocity;

    // theta_adjusted = theta + v_x*weights_control_velocity
    stateEstimate[7] = stateEstimate[7] + x_control(0) * MissionConstants::weights_control_velocity;

    // Set velocity to zero (it doesn't work very well in this LQR implementation)
    x_control(0) = 0;
    x_control(1) = 0;
    x_control.segment(4, 2) = stateEstimate.segment(6, 2);
    x_control.segment(6, 2) = stateEstimate.segment(9, 2);

    // Extract roll and pitch from stateEstimate, and put current integral step into euler_queue
    std::vector<double> currentIntegralStep = {stateEstimate(6) * loopTime, stateEstimate(7) * loopTime};
    euler_queue.push_back(currentIntegralStep);

    // Determine if euler_queue apprahced its limit, and if so, delelete its first entry
    if (euler_queue.size() > maxEulerEntries)
    {
        euler_queue.erase(euler_queue.begin());
    }
    // Create a vector that will hold the sums of all of the roll and pitch entries in euler_queue
    std::vector<double> euler_sum{0.0, 0.0};
    for (unsigned int i = 0; i < euler_queue.size(); i++)
    {
        euler_sum[0] += euler_queue[i][0];
        euler_sum[1] += euler_queue[i][1];
    }

    // Populate the second and third element with the integrals of roll and pitch
    x_control[2] = euler_sum[0];
    x_control[3] = euler_sum[1];

    if (current_iteration_index < MissionConstants::kNumberControllerGains - 1)
    {
        GetNextController_Gain_Time_Index(current_time);
    }
    // std::cout << "Current time: " << std::to_string(current_time) << " Nexttvc time: " <<std::to_string(next_tvc_time) << std::endl;
    // Calculate what angle we need to tell the tvc to move
    if (current_time > next_tvc_time)
    {
        // std::cout << "Calc input" << std::endl;
        // Calculate what angle we need to tell the tvc to move
        CalculateInput();
        next_tvc_time += MissionConstants::TVCPeriod;
    }
}

// shut down rocket functions
void Controller::UpdateSafe()
{
    // TODO. Center TVC, turn off reaction wheel, etc.
}

void Controller::GetNextController_Gain_Time_Index(double current_time)
{
    // Determine if a certain amount of time has passed, and if so, then increase the current_iteration_index and get the next K value

    double switch_time = (controller_gain_times[current_iteration_index + 1] + controller_gain_times[current_iteration_index]) / 2.0;
    if (current_time - k_iteration_start_time > switch_time)
    {
        current_iteration_index++;
    }
}

void Controller::CalculateInput()
{
    static double offset1;
    static double offset2;

    // This calculates u = -Kx

    input = controller_gains.block(current_iteration_index * 2, 0, 2, 8) * x_control;
    if (input.norm() > MissionConstants::kMaximumTvcAngle)
    {
        input = input * MissionConstants::kMaximumTvcAngle / input.norm();
    }

    offset1 = offset1 + MissionConstants::weights_control_steady_state * input(0) * loopTime;
    offset2 = offset2 + MissionConstants::weights_control_steady_state * input(1) * loopTime;

    // Figures out what angle we need to move the servos and then set them
    // [TODO] Move to hardware tvc_angles = TvcMath(input);
    tvc.SetTVCX(input(0) + offset1);
    tvc.SetTVCY(input(1) + offset2);
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

void Controller::ImportControlParameters(std::string file_name)
{
    // Imports the kmatrix file into controller_gains and the time values into controller_gain_times

    char separator = ',';
    std::string row, item;
    std::ifstream in(file_name);
    std::getline(in, row);
    std::stringstream iterationTimeStringStream(row);

    // Get the iteration times of the k-matrix
    for (int i = 0; i < 10; i++)
    {
        std::getline(iterationTimeStringStream, item, separator); // This gets values delimited by commas in the string

        controller_gain_times[i] = stod(item);
    }

    // Get the controller values of the k-matrix
    for (int i = 0; i < 2 * MissionConstants::kNumberControllerGains; i++)
    {
        std::getline(in, row);
        std::stringstream controllerValueStringStream(row);
        for (int j = 0; j < 8; j++)
        {
            std::getline(controllerValueStringStream, item, separator);
            controller_gains(i, j) = stod(item);
        }
    }

    in.close();
}

// When we go into landing mode, reset the k_iteration_start_time and current_iteration_index
void Controller::ResetKIteration(double current_time)
{
    next_tvc_time = current_time;
    k_iteration_start_time = current_time;
    current_iteration_index = 0;
}

Eigen::Matrix<double, 2, 8> Controller::GetCurrentKMatrix()
{
    return controller_gains.block(current_iteration_index * 2, 0, 2, 8);
}

int Controller::GetCurrentIterationIndex()
{
    return current_iteration_index;
}

Eigen::Matrix<double, 2, 1> Controller::GetCurrentTVCCommand()
{
    return input;
}