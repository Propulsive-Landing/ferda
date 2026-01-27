#pragma once

#include "Barometer.hpp"
#include "IMU.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"
#include "Controller.hpp"
#include "Igniter.hpp"
#include "ValveControl.hpp"
#include "SparkPlug.hpp"
#include "PressureTransducer.hpp"
#include "LoadCell.hpp"

class Mode
{
public:
    enum Phase
    {
        Calibration,
        TestTVC,
        Idle,
        Launch,
        Freefall,
        Terminate,
        Safe,
        Land,
        // Liquid Propulsion States
        HotfireIdle,
        ASITest,
        WaterFlow
    };

    Mode(Mode::Phase eInitialMode);
    bool Update(Navigation &navigation, Controller &controller, Igniter &igniter, IMU &imu, ValveControl &valveControl,
                SparkPlug &sparkPlug, PressureTransducer &pressureTransducer, LoadCell &loadCell);
    std::string LaunchKMatrix;
    std::string LandKMatrix;

private:
    Mode::Phase eCurrentMode;

    Mode::Phase UpdateCalibration(Navigation &navigation, Controller &controller, double currentTime);
    void GetGyroBiasOffset(Navigation &navigation, Controller &controller, IMU &imu, double currentTime);
    void GetAccelBiasOffset(Navigation &navigation, Controller &controller, IMU &imu, double currentTime);
    Mode::Phase UpdateTestTVC(Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateIdle(Navigation &navigation, Controller &controller, IMU &imu, double currentTime);
    Mode::Phase UpdateLaunch(Navigation &navigation, Controller &controller, Igniter &igniter, double current_time);
    Mode::Phase UpdateFreefall(Navigation &navigation, Controller &controller, Igniter &igniter, double currentTime);
    Mode::Phase UpdateLand(Navigation &navigation, Controller &controller, double current_time, Igniter &igniter);
    Mode::Phase UpdateSafeMode(Navigation &navigation, Controller &controller, double currentTime);
    void UploadKmatrices();
    // Liquid Propulsion State Updates
    Mode::Phase UpdateHotfireIdle(Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug);
    Mode::Phase UpdateASITest(Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug, double currentTime);
    Mode::Phase UpdateWaterFlow(Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug, double currentTime);
};
