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
    double k_iteration_start_time;
    // Eigen::Vector2d tvc_angles; [TODO MOVE TO HARDWARE]
    int current_iteration_index = 0;
    Eigen::Quaterniond q_current;
    Eigen::Quaterniond q_error;
    Eigen::Vector2d error_integral = Eigen::Vector2d::Zero();
    double height_error_integral = 0.0;
    Eigen::Vector2d translation_error_integral = Eigen::Vector2d::Zero();
    Eigen::Vector2d setpoint_angles = Eigen::Vector2d::Zero(); // [roll, pitch] from translation controller
    Eigen::Vector2d setpoint_angles_prev = Eigen::Vector2d::Zero(); // Previous setpoint angles for derivative term

public:
    TVC &tvc;
    Engine &engine;
    Eigen::Vector2d input;
    double loopTime = 0.005;
    double refPositionX = 0.0;      // reference position in x (meters)
    double refPositionY = 0.0;      // reference position in y (meters)
    double refPositionZ = 0.0;      // reference altitude / position in z (meters)
    double refVelocityX = 0.0;      // reference velocity in x (m/s)
    double refVelocityY = 0.0;      // reference velocity in y (m/s)
    double refVelocityZ = 0.0;      // reference velocity in z (m/s, positive is up)
    double refAccelerationZ = 0.0;   // feedforward acceleration in z (m/s^2, positive is up)
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
