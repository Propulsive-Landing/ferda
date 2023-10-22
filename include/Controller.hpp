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


public:
    Controller(TVC& tvc, Igniter& igniter);

    void UpdateLaunch(Navigation& navigation);
    void UpdateLand();
    void UpdateSafe();
};