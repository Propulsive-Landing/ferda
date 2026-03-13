#pragma once

#include "IMU.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"
#include "Controller.hpp"
#include "Igniter.hpp"
#include "LaunchManager.hpp"
#include "GPS.hpp"

class Mode
{
public:
    enum Phase
    {
        Calibration,
        TestTVC,
        Idle,
        Launch,
        Land,
        Terminate,
        Safe
    };

    // LaunchManager encapsulates the launch sub-mode state machine

    Mode(Mode::Phase eInitialMode);
    bool Update(Navigation &navigation, Controller &controller, GPS &gps, Igniter &igniter, IMU &imu);
    std::string AngleKMatrix;
    std::string HeightKMatrix;
    std::string TranslationKMatrix;

    // Setters for current acceleration/deceleration (should be updated by hardware telemetry)
    void SetCurrentMaxAcceleration(double accel) { launchManager.SetCurrentMaxAcceleration(accel); }
    void SetCurrentMaxDeceleration(double decel) { launchManager.SetCurrentMaxDeceleration(decel); }

private:
    Mode::Phase eCurrentMode;

    // Launch manager instance
    LaunchManager launchManager;

    Mode::Phase UpdateCalibration(Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateTestTVC(Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateIdle(Navigation &navigation, Controller &controller, IMU &imu, double currentTime);
    Mode::Phase UpdateLaunch(Navigation &navigation, Controller &controller, Igniter &igniter, double current_time);
    Mode::Phase UpdateSafeMode(Navigation &navigation, Controller &controller, double currentTime);
    void UploadKmatrices();
};
