#pragma once

#include <vector>
#include <Eigen/Dense>

#include "Barometer.hpp"
#include "IMU.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"

class Controller{

//Constant to be figured out later
//int kNumberControllerGains = 10;

private:
    TVC tvc;
    std::vector<std::vector<double>> euler_queue;
    Eigen::Matrix<double, 8, 1> x_control;
    Eigen::Matrix<double, 10*2, 8> controller_gains; 
    double next_tvc_time;
    double tvc_start_time;
    std::vector<float> controller_gain_times;
    Eigen::Vector2d input;
    Eigen::Vector2d tvc_angles;
    int current_iteration_index = 0;


public:
    Controller(TVC& tvc);   
    void UpdateLaunch(Navigation& navigation, double current_time);
    void UpdateTestTVC(double testTime);
    void UpdateLand();
    void UpdateSafe();
    void CalculateK(double startTime);
    void CalculateInput();
    Eigen::Vector2d TvcMath(Eigen::Vector2d input);
    void Start(double current_time);
    void Center();

    void ImportControlParameters(std::string file_name);

};