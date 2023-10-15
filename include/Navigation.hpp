#pragma once

#include <Eigen/Dense>

#include "IMU.hpp"
#include "Barometer.hpp"
#include "TVC.hpp"
#include "Igniter.hpp"

class Navigation
{
    private:
        IMU& imu;
        Barometer& barometer;
        TVC& tvc;
        Igniter& igniter;

        Eigen::Matrix<double, 12, 1> currentNavigation;
    public:
        Navigation(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter);
        Eigen::Matrix<double, 12, 1> GetNavigation(); // Defintion of navigation matrix: TODO (determine dimensions and document form)
        void UpdateNavigation(); // Defintion updates: TODO (determine dimensions and document form)
};