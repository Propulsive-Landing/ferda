#pragma once

#include "IMU.hpp"
#include "Camera.hpp"
#include "Magnetometer.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"
#include "Controller.hpp"
#include "Igniter.hpp"
#include "LaunchManager.hpp"
#include "GPS.hpp"
#include "ValveControl.hpp"
#include "SparkPlug.hpp"
#include "PressureTransducer.hpp"
#include "LoadCell.hpp"
#include "RF.hpp"

class Mode
{
public:
    enum Phase
    {
        Calibration,
        ActuatorCalibration,
        TestTVC,
        ChirpTVC,
        Idle,
        Launch,
        Land,
        Terminate,
        Safe,
        HotfireIdle,
        ASITest,
        WaterFlow
    };

    Mode(Mode::Phase eInitialMode);
    bool Update(Navigation &navigation, Controller &controller, GPS &gps, Igniter &igniter, IMU &imu,
                Magnetometer &magnetometer, ValveControl &valveControl, SparkPlug &sparkPlug, PressureTransducer &pressureTransducer,
                LoadCell &loadCell, Camera &camera);
    std::string AngleKMatrix;
    std::string HeightKMatrix;
    std::string TranslationKMatrix;

private:
    Mode::Phase eCurrentMode;

    // Launch manager instance
    LaunchManager launchManager;

    Mode::Phase UpdateCalibration(Navigation &navigation, Controller &controller, GPS &gps, Camera &camera, Magnetometer &magnetometer, double currentTime);
    Mode::Phase UpdateActuatorCalibration(Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateTestTVC(Navigation &navigation, Controller &controller, GPS &gps, Camera &camera, Magnetometer &magnetometer, double currentTime);
    Mode::Phase UpdateChirpTVC(Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateIdle(Navigation &navigation, Controller &controller, IMU &imu, GPS &gps, Camera &camera, Magnetometer &magnetometer, double currentTime);
    Mode::Phase UpdateLaunch(Navigation &navigation, Controller &controller, Igniter &igniter, float current_time);
    Mode::Phase UpdateSafeMode(Navigation &navigation, Controller &controller, double currentTime);
    void UploadKmatrices();
    // Liquid Propulsion State Updates
    Mode::Phase UpdateHotfireIdle(Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug, GPS &gps, Camera &camera, Magnetometer &magnetometer);
    Mode::Phase UpdateASITest(Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug, double currentTime);
    Mode::Phase UpdateWaterFlow(Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug, double currentTime);
    void CheckForToggleSensorCommands(RF::Command &command, GPS &gps, Camera &camera, Magnetometer &magnetometer);
};
