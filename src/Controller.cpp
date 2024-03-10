#include <Eigen/Dense>

#include "Controller.hpp"
#include "Telemetry.hpp"
#include "MissionConstants.hpp"
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

// constants to be figured out later 
namespace {
    double control_integral_period = 0.25;
    double fsw_loop_time = 1;
    double kPi = 3.1415926535897932384626433;
    double kDeg2Rad = kPi/180;
    double kRad2Deg = 180/kPi;
    double kMaximumTvcAngle = 7.5*kDeg2Rad;
    double kDeg2PulseWidth = ((double) 1000.0)/((double) 90.0);
    double kTvcXCenterPulseWidth = 1529;
    double kTvcYCenterPulseWidth = 915;
    int kNumberControllerGains = 10;
    double tvcPeriod = 0.02;
}


Controller::Controller(TVC& tvc) : tvc(tvc), x_control(Eigen::Matrix<double, 8, 1>::Zero()){}

void Controller::Start(double current_time){
    // Initialize variables 

    tvc_start_time = current_time;
    next_tvc_time = current_time;
}


void Controller::UpdateLaunch(Navigation& navigation, double current_time) {
    // Calculate desired control inputs for launch and actuate all control surfaces accordingly
     // Calculate desired control inputs for launch and actuate all control surfaces accordingly
    
    // Create a variable to determine the max amount of Euler Entries;
    int maxEulerEntries = control_integral_period/loopTime;

    // Create a matrix to store the returned stateEstimate from getNavigation() and store the yaw value into a varible
    Eigen::Matrix<double,12,1> stateEstimate = navigation.GetNavigation();
    double yaw = stateEstimate(8);

    // Calculate the rotation matrix to translate the body frame to the ground frame
    Eigen::Matrix2d rotation;
    rotation << cos(-yaw), -sin(yaw), sin(-yaw), cos(-yaw);

    // Populate x_control so that the first 2 entries are the stateEstimates' x and y velocities, its 4-5 entries are stateEstimates' roll and pitch values
    // and it's last 2 entries are stateEstimates' roll and pitch angular velocities.
    x_control.segment(0,2) = rotation * stateEstimate.segment(3,2);
    x_control.segment(4,2) = stateEstimate.segment(6,2);
    x_control.segment(6,2) = stateEstimate.segment(9,2);

    // Extract roll and pitch from stateEstimate, and put it into euler_queue
    std::vector<double> currentAngle {stateEstimate(6), stateEstimate(7)};
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
    x_control(2) = euler_sum[0] * loopTime;
    x_control(3) = euler_sum[1] * loopTime;

    if(current_iteration_index < kNumberControllerGains - 1){
        CalculateK(current_time);
    }

    if (current_time > next_tvc_time){
        // Calculate what angle we need to tell the tvc to move
        CalculateInput();
        next_tvc_time += tvcPeriod;


    }
}

void Controller::CalculateK(double current_time){
    // Determine if a certain amount of time has passed, and if so, then increase the current_iteration_index and get the next K value

    double switch_time = (controller_gain_times[current_iteration_index+1] - controller_gain_times[current_iteration_index]) / 2.0;
    if(current_time - tvc_start_time > switch_time)
    {
        std::cout<<"EnteredCalulateK"<<"\n";
        current_iteration_index++;
    }
}

void Controller::CalculateInput(){
    // This calculates u = -Kx
    std::cout<<"EnteredCalulcateInput"<<"\n";
    input = controller_gains.block(current_iteration_index*2, 0, 2, 8)*x_control;
    std::cout<<input<<"\n";
    if(input.norm() > kMaximumTvcAngle){
        input = input*kMaximumTvcAngle/input.norm();
    }
    // Figures out what angle we need to move the servos and then set them
    tvc_angles = TvcMath(input);
    tvc.SetXServo(tvc_angles(0));
    tvc.SetYServo(tvc_angles(1));
}


Eigen::Vector2d Controller::TvcMath(Eigen::Vector2d input){
    // Figures out what angle we need to move the servos
    Eigen::Vector2d output;

    input = input * kRad2Deg;

    output(0) = -.000095801*powf(input(0), 4) - .0027781*powf(input(0), 3) + .0012874*powf(input(0), 2) - 3.1271*input(0) -16.129;
    output(1) = - .0002314576*powf(input(1), 4) - .002425139*powf(input(1), 3) - .01204116*powf(input(1), 2) - 2.959760*input(1) + 57.18794;
    
   // std::cout<<output<<"\n";
    return output;
}

void Controller::Center(){
    // Center the tvc 

    input(0) = 0;
    input(1) = 0;
    tvc_angles = TvcMath(input);
    tvc.SetXServo(tvc_angles(0));
    tvc.SetYServo(tvc_angles(1));
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

