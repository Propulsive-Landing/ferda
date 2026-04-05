#pragma once

#include <Eigen/Dense>
#include <deque>
#include <vector>
#include <tuple>
#include <fstream>
#include "IMU.hpp"
#include "Magnetometer.hpp"
#include "Camera.hpp"
#include "GPS.hpp"
#include "Lidar.hpp"
#include "TVC.hpp"

class Navigation
{
private:
    IMU &imu;
    Magnetometer &magnetometer;
    GPS &gps;
    Lidar &lidar;
    Camera &camera;
    TVC &tvc;
    Eigen::Matrix<double, 16, 1> stateMat;
    std::deque<std::vector<double>> d_theta_queue_reckon;
    double pressureInit;
    std::tuple<double, double, double> linearAcceleration;
    std::tuple<double, double, double> angularRate;
    std::tuple<double, double, double> magneticField;
    std::tuple<double, double, double> gpsPosition;
    std::tuple<double, double> gpsVelocity;
    bool gpsAvailable;
    void magnetometerUpdate(const Eigen::Vector3d& magneticField, const Eigen::Matrix3d& R);
    void gpsUpdate(const Eigen::Vector3d& gpsPosition, const Eigen::Vector2d& gpsVelocity);
    void lidarUpdate(double lidar, const Eigen::Matrix3d& R);
    void cameraUpdate(const std::vector<Eigen::Vector3d>& cameraDirections, const Eigen::Matrix3d& R);
    std::ofstream dataFile;
    bool onPad = true; // Flag to indicate if rocket is on the pad (idle mode)
    double estimatedMassKg = 0.0;
    double estimatedMassFraction = 1.0;
    Eigen::Vector3d estimatedCenterOfMassBodyM = Eigen::Vector3d::Zero();
    Eigen::Vector3d estimatedMomentOfInertiaBodyKgm2 = Eigen::Vector3d::Zero();
    int gps_update_counter = 0;
    static constexpr int kGPSUpdateCadence = 20; // Update GPS every N nav steps
    int lidar_update_counter = 0;
    static constexpr int kLidarUpdateCadence = 4; // Update lidar every N nav steps
    int magnetometer_update_counter = 0;
    static constexpr int kMagnetometerUpdateCadence = 2; // Update magnetometer every N nav steps
    double camera_capture_elapsed_s = 0.0;
    static constexpr double kCameraCapturePeriodS = 0.2; // 5 Hz capture requests
    double last_camera_frame_id = -1.0;

public:
    double loopTime = 0.005;
    Navigation(IMU &imu, Magnetometer &magnetometer, GPS &gps, Lidar &lidar, Camera &camera, TVC &tvc);
    void reset();
    Eigen::MatrixXd P;
    Eigen::Matrix<double, 16, 1> GetNavigation(); // Defintion of state matrix: TODO (determine dimensions and document form)
    void UpdateNavigation();                      // Defintion updates: TODO (determine dimensions and document form)
    void padUpdateVelocity();
    void padUpdateAngularVelocity(const Eigen::Vector3d& angularVelocity);
    void SetOnPad(bool isOnPad); // Set whether rocket is on the pad
    Eigen::Vector3d GetAngularVelocity();
    std::tuple<double, double, double> ComputeAngularRollingAverage(std::vector<double> d_theta_now);
    Eigen::Vector3d x_e, v_e;
    Eigen::Quaterniond q;
    Eigen::Vector3d a_b, w_b;
    Eigen::Vector3d w;
    Eigen::Matrix3d CreateRotationalMatrix(double phi, double theta, double psi);
    Eigen::Matrix3d skew(const Eigen::Vector3d &v);
    void kalmanUpdate(
        const Eigen::MatrixXd &H,
        const Eigen::MatrixXd &V,
        const Eigen::VectorXd &y,
        const Eigen::VectorXd &y_pred);
    std::tuple<double, double, double> GetLinearAcceleration();
    std::tuple<double, double, double> GetAngularAcceleration();
    std::tuple<double, double, double> GetMagneticField();
    std::tuple<double, double, double> GetGPSPosition();
    std::tuple<double, double> GetGPSVelocity();
    bool GPSAvailable();
    void UpdateMassFractionEstimate(double throttleCommandN);
    void UpdateMassPropertyEstimates();
    double GetEstimatedMassFraction();
    double GetEstimatedMassKg();
    Eigen::Vector3d GetEstimatedCenterOfMassBodyM();
    Eigen::Vector3d GetEstimatedMomentOfInertiaBodyKgm2();
};