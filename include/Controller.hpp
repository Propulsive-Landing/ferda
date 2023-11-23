#pragma once

#include <vector>
#include <Eigen/Dense>

#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"

class Controller{


private:
    TVC tvc;
    Igniter igniter;
    std::vector<double> IMU;
    Eigen::Matrix<double, 2, 1> lastCommand;

public:
    Controller(TVC& tvc, Igniter& igniter);

    Eigen::Matrix<double, 2, 1> stateMat;
    
    void UpdateGround(Navigation& navigation);
    void UpdateLaunch(Navigation& navigation);
    void UpdateLand();
    void UpdateSafe();
};