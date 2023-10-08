#pragma once
#include <vector>

class Controller{


private:
    TVC tvc;
    Igniter igniter;
    std::vector<double> IMU;


public:

    Controller(TVC tvc, Igniter igniter);

    void setTVCservos(double X, double Y);
    void ignite();
    void launch();


    void updateGround();
    void updateLaunch();
    void updateLand();
    void updateSafe();



};