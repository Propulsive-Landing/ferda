#include <Eigen/Dense>

#include "Controller.hpp"
#include "Telemetry.hpp"
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
}


Controller::Controller(TVC& tvc) : tvc(tvc), x_control(Eigen::Matrix<double, 8, 1>::Zero()){}

void Controller::Start(double current_time){
    // Initialize variables 

    tvc_start_time = current_time;
}


void Controller::UpdateTestTVC(double testTime) {
    double angle = sin(testTime)*90;

    tvc.SetXServo(angle);
    tvc.SetYServo(angle);
}

void Controller::UpdateLaunch(Navigation& navigation, double current_time) {
    // Calculate desired control inputs for launch and actuate all control surfaces accordingly
    Center();
}


//communicate with TVC
void Controller::UpdateLand(){
    // TODO. Calculate desired control inputs for land
    // TODO. Actuate all control surfaces accordingly
}

//shut down rocket functions
void Controller::UpdateSafe(){
    // TODO. Center TVC, turn off reaction wheel, etc.
}

void Controller::CalculateK(double current_time){
    // Determine if a certain amount of time has passed, and if so, then increase the current_iteration_index and get the next K value

    double switch_time = (controller_gain_times[current_iteration_index+1] - controller_gain_times[current_iteration_index]) / 2.0;
    if(current_time - tvc_start_time > switch_time)
    {
        current_iteration_index++;
    }
}

void Controller::CalculateInput(){
    // This calculates u = -Kx

    input = controller_gains.block(current_iteration_index*2, 0, 2, 8)*x_control;
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
    
    char separator = '\t';
    std::string row, item;
    std::ifstream in(file_name);
    std::ofstream out("test.csv");
    for (int i=0; i< 2*kNumberControllerGains; i++){
        std::getline(in, row);
        std::stringstream ss(row);
        for (int j=0; j<8; j++){
            std::getline(ss, item, separator);
            controller_gains(i, j) = stof(item);
        }
    }
    in.close();
}

