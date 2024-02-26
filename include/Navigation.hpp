#pragma once

#include <Eigen/Dense>
#include <deque>
#include <vector>
#include <tuple>
#include "IMU.hpp"
#include "Barometer.hpp"
#include "TVC.hpp"

class Navigation
{
    private:
        IMU& imu;
        Barometer& barometer;
        TVC& tvc;

        Eigen::Matrix<double, 12, 1> stateMat;
        std::deque<std::vector<double>> d_theta_queue_reckon;
        double pressureInit;

    public:
        double loopTime = 0.005;
        Navigation(IMU& imu, Barometer& barometer, TVC& tvc);
        Eigen::Matrix<double, 12, 1> GetNavigation(); // Defintion of state matrix: TODO (determine dimensions and document form)
        void UpdateNavigation(); // Defintion updates: TODO (determine dimensions and document form)
        void Start();
        std::tuple<double,double,double> ComputeAngularRollingAverage();
        Eigen::Matrix3d CreateRotationalMatrix(double phi, double theta, double psi);
        double GetHeight();
        std::tuple<double, double, double> GetBodyAcceleration();
};