#pragma once

#include <Eigen/Dense>
#include <deque>
#include <vector>
#include <tuple>
#include "IMU.hpp"
#include "Magnetometer.hpp"
#include "Camera.hpp"
#include "GPS.hpp"
#include "TVC.hpp"

class Navigation
{
private:
    IMU &imu;
    Magnetometer &magnetometer;
    GPS &gps;
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
    void gpsUpdate(const Eigen::Vector3d& gpsPosition);
    void cameraUpdate(const Eigen::VectorXd& cameraDirectionsVector, const Eigen::Matrix3d& R);
    std::tuple<double> magnetometerAvailable;
    std::tuple<double> gpsAvailable;
    std::tuple<double> cameraAvailable;

public:
    double loopTime = 0.005;
    Navigation(IMU &imu, Magnetometer &magnetometer, GPS &gps, Camera &camera, TVC &tvc);
    void reset();
    Eigen::MatrixXd P;
    Eigen::Matrix<double, 16, 1> GetNavigation(); // Defintion of state matrix: TODO (determine dimensions and document form)
    void UpdateNavigation();                      // Defintion updates: TODO (determine dimensions and document form)
    std::tuple<double, double, double> ComputeAngularRollingAverage(std::vector<double> d_theta_now);
    Eigen::Vector3d x_e, v_e;
    Eigen::Quaterniond q;
    Eigen::Vector3d a_b, w_b;
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
};