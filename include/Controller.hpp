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

    void setTVCservos(double X, double Y);
    void ignite();
    void launch();


    void UpdateGround();
    void UpdateLaunch(Navigation& navigation); // what if we passed in reference to state vector instead of whole nav class?
    void UpdateLand();
    void UpdateSafe();

    void HandleAborts(); // Method to handle any abort that comes in (might need enum of aborts)

};