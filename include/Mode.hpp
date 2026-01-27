#pragma once

#include "Barometer.hpp"
#include "IMU.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"
#include "Controller.hpp"
#include "Igniter.hpp"

class Mode
{
public:
    enum Phase
    {
        Calibration,
        TestTVC,
        Idle,
        Launch,
        Terminate,
        Safe,
        Land
    };

    Mode(Mode::Phase eInitialMode);
    bool Update(Navigation &navigation, Controller &controller, Igniter &igniter, IMU &imu);
    std::string LaunchKMatrix;
    std::string LandKMatrix;

private:
    Mode::Phase eCurrentMode;

    Mode::Phase UpdateCalibration(Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateTestTVC(Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateIdle(Navigation &navigation, Controller &controller, IMU &imu, double currentTime);
    Mode::Phase UpdateLaunch(Navigation &navigation, Controller &controller, Igniter &igniter, double current_time);
    Mode::Phase UpdateLand(Navigation &navigation, Controller &controller, double current_time, Igniter &igniter);
    Mode::Phase UpdateSafeMode(Navigation &navigation, Controller &controller, double currentTime);
    void UploadKmatrices();
};
