#pragma once

#include <Eigen/Dense>
#include <deque>
#include <vector>

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

        Eigen::Matrix<double, 12, 1> stateMat;
       
        int count;
        std::deque<std::vector<double>> d_theta_queue_reckon;

    public:
        Navigation(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter);
        Eigen::Matrix<double, 12, 1> GetNavigation(); // Defintion of state matrix: TODO (determine dimensions and document form)
        void UpdateNavigation(); // Defintion updates: TODO (determine dimensions and document form)
        void Reset();
};