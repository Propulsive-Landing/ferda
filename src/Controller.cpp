#include <Eigen/Dense>


#include "Controller.hpp"
#include <cmath>

// Constant to be found later
double control_integral_period = 1;
double fsw_loop_time = 1;

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