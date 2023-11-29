#pragma once

#include <vector>
#include <Eigen/Dense>

#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"

class Controller{

//Constant to be figured out later
//int numberControllerGains = 2;

private:
    TVC tvc;
    Igniter igniter;
    std::vector<double> IMU;
    Eigen::Matrix<double, 2, 1> lastCommand;
    std::vector<std::vector<double>> euler_queue;
    Eigen::Matrix<double, 8, 1> x_control;
    Eigen::Matrix<double, 1*2, 8> controller_gains; 
    int current_iteration_index = 0;
    float next_tvc_time;
    float tvc_start_time;
    std::vector<float> controller_gain_times;


public:
    Controller(TVC& tvc, Igniter& igniter);
    
    Eigen::Matrix<double, 2, 1> stateMat;
    
    void UpdateLaunch(Navigation& navigation);
    void UpdateLand();
    void UpdateSafe();
    void calculateK(double startTime);
    void calculateInput();
    Eigen::Vector2d tvcMath(Eigen::Vector2d input);
    void start();
};