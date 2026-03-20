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
    std::tuple<double, double, double, double, double, double, double, double, double> cameraDirections;
    void magnetometerUpdate(const Eigen::Vector3d& magneticField, const Eigen::Matrix3d& R);
    void gpsUpdate(const Eigen::Vector3d& gpsPosition, const Eigen::Vector2d& gpsVelocity);
    void lidarUpdate(double lidar, const Eigen::Matrix3d& R);
    void cameraUpdate(const Eigen::VectorXd& cameraDirectionsVector, const Eigen::Matrix3d& R);
    std::tuple<double> magnetometerAvailable;
    std::tuple<double> gpsAvailable;
    std::tuple<double> cameraAvailable;
    std::ofstream dataFile;
    bool onPad = true; // Flag to indicate if rocket is on the pad (idle mode)
    double estimatedMassKg = 0.0;
    double estimatedMassFraction = 1.0;
    Eigen::Vector3d estimatedCenterOfMassBodyM = Eigen::Vector3d::Zero();
    Eigen::Vector3d estimatedMomentOfInertiaBodyKgm2 = Eigen::Vector3d::Zero();

public:
    double loopTime = 0.005;
    Navigation(IMU &imu, Magnetometer &magnetometer, GPS &gps, Lidar &lidar, Camera &camera, TVC &tvc);
    void reset();
    Eigen::MatrixXd P;
    Eigen::Matrix<double, 16, 1> GetNavigation(); // Defintion of state matrix: TODO (determine dimensions and document form)
    void UpdateNavigation();                      // Defintion updates: TODO (determine dimensions and document form)
    void padUpdatePosition();
    void padUpdateAngularVelocity(const Eigen::Vector3d& w);
    void SetOnPad(bool isOnPad); // Set whether rocket is on the pad
    Eigen::Vector3d GetAngularVelocity();
    std::tuple<double, double, double> ComputeAngularRollingAverage(std::vector<double> d_theta_now);
    Eigen::Vector3d x_e, v_e;
    Eigen::Quaterniond q;
    Eigen::Vector3d a_b, w_b;
    Eigen::Vector3d w;
    Eigen::Matrix3d CreateRotationalMatrix(double phi, double theta, double psi);
    Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    void kalmanUpdate(
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& V,
        const Eigen::VectorXd& y,
        const Eigen::VectorXd& y_pred);
    std::tuple<double, double, double> GetLinearAcceleration();
    std::tuple<double, double, double> GetAngularAcceleration();
    std::tuple<double, double, double> GetMagneticField();
    std::tuple<double, double, double> GetGPSPosition();
    std::tuple<double> MagnetometerAvailable();
    std::tuple<double> GPSAvailable();
    std::tuple<double> CameraAvailable();
    std::tuple<double, double, double, double, double, double, double, double, double> GetUnitVectors();
    void UpdateMassFractionEstimate(double throttleCommandN);
    void UpdateMassPropertyEstimates();
    double GetEstimatedMassFraction();
    double GetEstimatedMassKg();
    Eigen::Vector3d GetEstimatedCenterOfMassBodyM();
    Eigen::Vector3d GetEstimatedMomentOfInertiaBodyKgm2();
};