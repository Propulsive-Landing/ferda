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
        Standby,
        Calibration,
        ActuatorCalibration,
        TestTVC,
        ChirpTVC,
        Idle,
        Launch,
        Land,
        Terminate,
        Abort,
        Safe,
        HotfireIdle,
        ASITest,
        WaterFlow,
        ThreeSecondHotfire
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

    Mode::Phase UpdateStandby(RF::Command &command, Controller &controller);
    Mode::Phase UpdateCalibration(RF::Command &command, Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateActuatorCalibration(RF::Command &command, Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateTestTVC(RF::Command &command, Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateChirpTVC(Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateIdle(RF::Command &command, Navigation &navigation, Controller &controller, double currentTime);
    Mode::Phase UpdateLaunch(RF::Command &command, Navigation &navigation, Controller &controller, Igniter &igniter, float current_time);
    Mode::Phase UpdateSafeMode(Navigation &navigation, Controller &controller, double currentTime);
    void UploadKmatrices();
    // Liquid Propulsion State Updates
    Mode::Phase UpdateHotfireIdle(RF::Command &command, Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug);
    Mode::Phase UpdateASITest(RF::Command &command, Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug, double currentTime);
    Mode::Phase UpdateWaterFlow(RF::Command &command, Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug, double currentTime);
    Mode::Phase Update3SecondHotfire(RF::Command &command, Navigation &navigation, Controller &controller, ValveControl &valveControl, SparkPlug &sparkPlug, double currentTime);
    Mode::Phase UpdateAbort(ValveControl &valvecontrol, SparkPlug &sparkplug);
    void CheckForToggleSensorCommands(RF::Command &command, GPS &gps, Camera &camera, Magnetometer &magnetometer);
    void CloseAllValvesAndSparkPlug(ValveControl &valveControl, SparkPlug &sparkPlug);
};
