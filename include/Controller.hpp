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
//int numberControllerGains = 1;

private:
    TVC tvc;
    Igniter igniter;
    std::vector<std::vector<double>> euler_queue;
    Eigen::Matrix<double, 8, 1> x_control;
    Eigen::Matrix<double, 1*2, 8> controller_gains; 
    int current_iteration_index = 0;
    float next_tvc_time;
    float tvc_start_time;
    std::vector<float> controller_gain_times;
    Eigen::Vector2d input;
    Eigen::Vector2d tvc_angles;



public:
    Controller(TVC& tvc, Igniter& igniter);    

    void UpdateLaunch(Navigation& navigation);
    void UpdateIdle(Navigation& navigation);
    void UpdateTestTVC(double testTime);
    void UpdateLand();
    void UpdateSafe();

    void CalculateK(double startTime);
    void CalculateInput();
    Eigen::Vector2d TvcMath(Eigen::Vector2d input);
    void Start();
    void Center();
    void HandleAborts(int abort);

};