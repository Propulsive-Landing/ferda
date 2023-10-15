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
    public:
        Navigation(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter);
        Eigen::Matrix<double, 3, 1> GetNavigation(); // Defintion of navigation matrix: TODO (determine dimensions and document form)
        void UpdateNavigation(Eigen::Matrix<double, 4, 1> updates); // Defintion updates: TODO (determine dimensions and document form)
};