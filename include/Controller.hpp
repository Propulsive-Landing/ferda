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
    std::vector<float> controller_gain_times = {0, 0.025, 0.0575, 0.265, 0.59, 0.8575, 1.0450, 1.1475, 1.3425, 1.9175};
    Eigen::Vector2d input;
    Eigen::Vector2d tvc_angles;
    int current_iteration_index = 0;

public:
    double loopTime = 0.001;
    Controller(TVC& tvc);   
    void UpdateLaunch(Navigation& navigation, double current_time);
    void UpdateTestTVC(double testTime);
    void UpdateLand(Navigation& navigation, double current_time);
    void UpdateSafe();
    void CalculateK(double startTime);
    void CalculateInput();
    void stabilizeAtOffset(Navigation& navigation, double current_time, double offset);
    Eigen::Vector2d TvcMath(Eigen::Vector2d input);
    void Start(double current_time);
    void Center();

    void ImportControlParameters(std::string file_name);

};