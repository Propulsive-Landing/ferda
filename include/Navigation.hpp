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

    public:
        double loopTime = 0.005;
        double pressureInit;
        Navigation(IMU& imu, Barometer& barometer, TVC& tvc);
        void reset();
        Eigen::Matrix<double, 12, 1> GetNavigation(); // Defintion of state matrix: TODO (determine dimensions and document form)
        void UpdateNavigation(int i); // Defintion updates: TODO (determine dimensions and document form)
        std::tuple<double,double,double> ComputeAngularRollingAverage();
        std::vector<double> D_Theta_Now_Math(double phi, double theta, double psi, std::tuple<double,double,double> angularRate);
        Eigen::Matrix3d CreateRotationalMatrix(double phi, double theta, double psi);
        double GetHeight(int i);
        std::tuple<double, double, double> GetTestAcceleration(int i);
        std::tuple<double, double, double> GetTestGyroscope(int i);
        double GetTestBarom(int i);
        void importTestAccAndTestGyro();
        void importTestBarom();

        // Eigen::Matrix<double, 602, 3> linearAccels; // For testing
        // Eigen::Matrix<double, 602, 3> gyroAccels;
        std::vector<std::vector<double>> linearAccels;
        std::vector<std::vector<double>> gyroAccels;
        std::vector<std::vector<double>> baromValues;

};