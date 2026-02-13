#pragma once

#include <vector>
#include <Eigen/Dense>

#include "IMU.hpp"
#include "TVC.hpp"
#include "Engine.hpp"
#include "Navigation.hpp"
#include "MissionConstants.hpp"

class Controller
{

private:
    Eigen::Matrix<double, 6, 1> x_control;
    Eigen::Matrix<double, 2, 6> angle_controller_gains;
    Eigen::Matrix<double, 1, 3> height_controller_gains;
    Eigen::Matrix<double, 2, 6> translation_controller_gains;
    double next_tvc_time;
    double k_iteration_start_time;
    // Eigen::Vector2d tvc_angles; [TODO MOVE TO HARDWARE]
    int current_iteration_index = 0;
    Eigen::Quaterniond q_current;
    Eigen::Quaterniond q_error;
    Eigen::Vector3d error_integral = Eigen::Vector3d::Zero();
    double height_error_integral = 0.0;

public:
    TVC tvc;
    Engine engine;
    Eigen::Vector2d input;
    double loopTime = 0.005;
    double refPositionZ = 0.0;      // reference altitude / position in z (meters)
    double refVelocityZ = 0.0;      // reference velocity in z (m/s, positive is up)
    Controller(TVC &tvc, Engine &engine);
    void UpdateLaunch(Navigation &navigation, double current_time);
    void UpdateTestTVC(double testTime);
    void AttitudeControl(Navigation &navigation);
    void HeightControl(Navigation &navigation);
    void TranslationControl(Navigation &navigation);
    void UpdateSafe();
    void GetNextController_Gain_Time_Index(double startTime);
    void CalculateInput();
    Eigen::Vector2d TvcMath(Eigen::Vector2d input);
    void Start(double current_time);
    void Center();
    void ImportAngleParameters(std::string file_name);
    void ImportHeightParameters(std::string file_name);
    void ImportTranslationParameters(std::string file_name);
    int GetCurrentIterationIndex();
    Eigen::Matrix<double, 2, 1> GetCurrentTVCCommand();
};
