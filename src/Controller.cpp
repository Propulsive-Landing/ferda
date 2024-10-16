#include <Eigen/Dense>

#include "Controller.hpp"
#include "Telemetry.hpp"
#include "MissionConstants.hpp"
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

Controller::Controller(TVC& tvc) : tvc(tvc), x_control(Eigen::Matrix<double, 8, 1>::Zero()){}

void Controller::Start(double current_time){
    // Initialize variables 

    tvc_start_time = current_time;
    next_tvc_time = current_time;
    ResetKIteration(current_time); // Sets iteration start time
}


void Controller::UpdateTestTVC(double testTime) {

    double angleA = sin(testTime)*MissionConstants::kMaximumTvcAngle; // Rad
    double angleB = cos(testTime)*MissionConstants::kMaximumTvcAngle; // Rad


    input(0) = angleA;
    input(1) = angleB;
    // [TOD] Move to hardware tvc_angles = TvcMath(input);

    tvc.SetTVCX(input(0));
    tvc.SetTVCY(input(1));
}

void Controller::UpdateLaunch(Navigation &navigation, double current_time){
    //Use the TVC to stabilize the rocket for landing

    stabilizeAtOffset(navigation, current_time, 0);
//     stabilizeAtOffset(navigation, current_time, 5*kDeg2Rad); TODO ADDRESS WHY THIS IS 

}


//communicate with TVC
void Controller::UpdateLand(Navigation &navigation, double current_time){
    //Use the TVC to stabilize the rocket for landing

    stabilizeAtOffset(navigation, current_time, 0);
}

void Controller::stabilizeAtOffset(Navigation& navigation, double current_time, double offset) 
{

    // Calculate desired control inputs for launch and actuate all control surfaces accordingly

    // Create a variable to determine the max amount of Euler Entries;
    int maxEulerEntries = MissionConstants::kControlIntegralPeriod/loopTime;

    // Create a matrix to store the returned stateEstimate from getNavigation() and store the yaw value into a varible
    Eigen::Matrix<double,12,1> stateEstimate = navigation.GetNavigation();
   
   /* WE DON'T NEED THIS CODE FOR RIGHT NOW. X AND Y VELOCITY WILL BE 0
    double yaw = stateEstimate(8);

     Calculate the rotation matrix to translate the body frame to the ground frame
    Eigen::Matrix2d rotation;
    rotation << cos(-yaw), -sin(-yaw), sin(-yaw), cos(-yaw);

     Populate x_control so that the first 2 entries are the stateEstimates' x and y velocities, its 4-5 entries are stateEstimates' roll and pitch values
     and it's last 2 entries are stateEstimates' roll and pitch angular velocities.
   
    x_control.segment(0,2) = rotation * stateEstimate.segment(3,2);
   */
  
    x_control(0) = 0;
    x_control(1) = 0;
    x_control.segment(4,2) = stateEstimate.segment(6,2);
    x_control.segment(6,2) = stateEstimate.segment(9,2);

    // Extract roll and pitch from stateEstimate, and put it into euler_queue
    std::vector<double> currentAngle = {stateEstimate(6), stateEstimate(7)};
    euler_queue.push_back(currentAngle);

    // Determine if euler_queue apprahced its limit, and if so, delelete its first entry
    if (euler_queue.size() > maxEulerEntries)
    {
        euler_queue.erase(euler_queue.begin());
    }
     // Create a vector that will hold the sums of all of the roll and pitch entries in euler_queue
    std::vector<double> euler_sum {0.0, 0.0};
    for(int i = 0; i < euler_queue.size(); i++)
    {
        euler_sum[0] += euler_queue[i][0];
        euler_sum[1] += euler_queue[i][1];
    }

    // Populate the second and third element with the integrals of roll and pitch
    x_control[2] = euler_sum[0] * loopTime;
    x_control[3] = euler_sum[1] * loopTime;


    if(current_iteration_index < MissionConstants::kNumberControllerGains - 1){
        GetNextController_Gain_Time_Index(current_time);
    }
    // std::cout << "Current time: " << std::to_string(current_time) << " Nexttvc time: " <<std::to_string(next_tvc_time) << std::endl;
    // Calculate what angle we need to tell the tvc to move
    if (current_time > next_tvc_time){
        // std::cout << "Calc input" << std::endl;
        // Calculate what angle we need to tell the tvc to move
        CalculateInput();
        next_tvc_time += MissionConstants::TVCPeriod;

    }
}

//shut down rocket functions
void Controller::UpdateSafe(){
    // TODO. Center TVC, turn off reaction wheel, etc.
}

void Controller::GetNextController_Gain_Time_Index(double current_time){
    // Determine if a certain amount of time has passed, and if so, then increase the current_iteration_index and get the next K value

    double switch_time = (controller_gain_times[current_iteration_index+1] + controller_gain_times[current_iteration_index]) / 2.0;
    if(current_time - k_iteration_start_time > switch_time)
    {
        current_iteration_index++;
    }
}

void Controller::CalculateInput(){
    // This calculates u = -Kx

    input = controller_gains.block(current_iteration_index*2, 0, 2, 8)*x_control;
    if(input.norm() > MissionConstants::kMaximumTvcAngle){
        input = input*MissionConstants::kMaximumTvcAngle/input.norm();
    }

    // Figures out what angle we need to move the servos and then set them
    // [TODO] Move to hardware tvc_angles = TvcMath(input);
    tvc.SetTVCX(input(0));
    tvc.SetTVCY(input(1));
}

void Controller::Center(){
    // Center the tvc 
    input(0) = 0;
    input(1) = 0;
    // [TODO] Move to hardware tvc_angles = TvcMath(input);
    tvc.SetTVCX(input(0));
    tvc.SetTVCY(input(1));
}

void Controller::ImportControlParameters(std::string file_name){
    // Imports the kmatrix file into controller_gains and the time values into controller_gain_times
    
    char separator = ',';
    std::string row, item;
    std::ifstream in(file_name);
    std::getline(in, row);
    std::stringstream ss(row);
    for (int i = 0; i < 10; i++){
         std::getline(ss, item, separator);
         controller_gain_times.push_back(stod(item));
    }

    for (int i=0; i < 2* MissionConstants::kNumberControllerGains; i++){
        std::getline(in, row);
        std::stringstream ss(row);
        for (int j=0; j<8; j++){
            std::getline(ss, item, separator);
            controller_gains(i, j) = stod(item);
        }
    }

    in.close();
}

void Controller::ResetKIteration(double current_time){
    k_iteration_start_time = current_time;
    current_iteration_index = 0;
}

Eigen::Matrix<double, 2, 8> Controller::GetCurrentKMatrix()
{
    return controller_gains.block(current_iteration_index*2, 0, 2, 8);
}
