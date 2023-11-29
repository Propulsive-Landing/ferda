#include <Eigen/Dense>
#include <vector>

#include "Controller.hpp"
#include <cmath>

// Constant to be found later
double control_integral_period = 1;
double fsw_loop_time = 1;
const float kPi = 3.1415926535897932384626433;
const float kDeg2Rad = kPi/180;
const float kRad2Deg = 180/kPi;
const float kMaximumTvcAngle = 7.5*kDeg2Rad;
const float kDeg2PulseWidth = ((float) 1000.0)/((float) 90.0);
const float kTvcXCenterPulseWidth = 1529;
const float kTvcYCenterPulseWidth = 915;

Controller::Controller(TVC& tvc, Igniter& igniter) : tvc(tvc), igniter(igniter) {}

void Controller::start(){
    x_control.setZero();
    next_tvc_time = 0;
    tvc_start_time = 0;
}

void Controller::UpdateLaunch(Navigation& navigation) {
    // TODO. Calculate desired control inputs for launch
    // TODO. Actuate all control surfaces accordingly
    int maxEulerEntries = control_integral_period/fsw_loop_time;

    Eigen::Matrix<double,12,1> stateEstimate = navigation.GetNavigation();
    double yaw = stateEstimate(8);

    Eigen::Matrix2d rotation;
    rotation << cos(-yaw), -sin(yaw), sin(-yaw), cos(-yaw);
    x_control.segment(0,2) = rotation * stateEstimate.segment(3,2);
    x_control.segment(4,2) = stateEstimate.segment(6,2);
    x_control.segment(6,2) = stateEstimate.segment(9,2);

    std::vector<double> currentAngle {stateEstimate(6), stateEstimate(7)};
    euler_queue.push_back(currentAngle);
    if (euler_queue.size() > maxEulerEntries)
    {
        euler_queue.erase(euler_queue.begin());
    }
    std::vector<double> euler_sum {0.0, 0.0};
    for(int i = 0; i < euler_queue.size(); i++)
    {
        euler_sum[0] += euler_queue[i][0];
        euler_sum[1] += euler_queue[i][1];
    }
    x_control[2] = euler_sum[0] * fsw_loop_time;
    x_control[3] = euler_sum[1] * fsw_loop_time;
    
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

void Controller::calculateInput(){
    input_ = controller_gains.block(current_iteration_index*2, 0, 2, 8)*x_control;
    if(input_.norm() > kMaximumTvcAngle){
        input_ = input_*kMaximumTvcAngle/input_.norm();
    }
    tvc_ = tvcMath(input_);
}

Eigen::Vector2d Controller::tvcMath(Eigen::Vector2d input){
    Eigen::Vector2d output;

    input = input * kRad2Deg;

    output(0) = -.000095801*powf(input(0), 4) - .0027781*powf(input(0), 3) + .0012874*powf(input(0), 2) - 3.1271*input(0) -16.129;
    output(1) = - .0002314576*powf(input(1), 4) - .002425139*powf(input(1), 3) - .01204116*powf(input(1), 2) - 2.959760*input(1) + 57.18794;
    //convert to pulse width
    output(0) = std::round(output(0)*kDeg2PulseWidth + kTvcXCenterPulseWidth);
    output(1) = std::round(output(1)*kDeg2PulseWidth + kTvcYCenterPulseWidth);
    return output;

}