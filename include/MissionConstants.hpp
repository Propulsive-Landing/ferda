#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <Eigen/Dense>

#ifndef __MISSION_CONSTANTS__

#define __MISSION_CONSTANTS__

namespace MissionConstants
{
    // YAML::Node LoadConstants(std::string filepath) {
    //     YAML::Node constants = YAML::LoadFile(filepath);
    //     return constants;
    // }

    // Physical constants
    const double kPi = 3.1415926535897932384626433;
    const double kGravity = 9.80298; // calculated at Ashford Town Park using https://www.sensorsone.com/local-gravity-calculator/#height
    const double kPressureH = 0.0;   // TODO: calculate H
    const double kRad2Deg = 180 / kPi;
    const double kDeg2Rad = kPi / 180;

    // Navigation constants
    const double kNavThetaDotSmooth = 0.05;
    const double kFswLoopTime = .005;
    const double kFSWCalibrationTime = 0.05;
    const double kVehicleWetMassKg = 94.0;
    const double kVehicleDryMassKg = 66.0;
    const double kThrottleToMassFlowScale = -5.6e-4; // kg/(N*s)
    const Eigen::Vector3d kVehicleWetCenterOfMassBodyM = Eigen::Vector3d(0.0, 0.0, 0.5);
    const Eigen::Vector3d kVehicleDryCenterOfMassBodyM = Eigen::Vector3d(0.0, 0.0, -0.5);
    const Eigen::Vector3d kVehicleWetMomentOfInertiaBodyKgm2 = Eigen::Vector3d(30.0, 30.0, 0.44);
    const Eigen::Vector3d kVehicleDryMomentOfInertiaBodyKgm2 = Eigen::Vector3d(20.0, 20.0, 0.34);
    const double originalOffsetAngle = 5 * kDeg2Rad;
    const Eigen::Vector3d kEarthMagField = Eigen::Vector3d(-4.8415717072661559, 20.116087207173326, -46.952237491806379);
    const Eigen::Vector3d kSensorCameraPosition = Eigen::Vector3d(0.2, 0.0, -1); // Position of the camera in the body frame (in meters)
    const Eigen::Vector3d kSensorCameraOrientationRad = Eigen::Vector3d(0.0, 2.3561944901923448, 0.0); // XYZ Euler orientation from camera frame to body frame
    const Eigen::Vector3d kSensorGPSPosition = Eigen::Vector3d(0.0, 0.0, 1.0); // Position of the GPS sensor (antenna) in the body frame (in meters)
    const Eigen::Vector3d kStructuresGroundOffset = Eigen::Vector3d(0.0, 0.0, -1.0); // Offset from marker data frame to ground frame
    const double kSensorCameraNoise = 1e-3;
    const double kNavCameraNoiseFactor = 1.5;
    inline const Eigen::Matrix<double, 3, 3> kMarkerData =
        (Eigen::Matrix<double, 3, 3>() <<
            2.0, 2.0, 5.0,
            1.7320508075688774, -1.7320508075688774, 0.0,
            0.0,  0.0,  0.0
        ).finished();

    // Controller constants, TODO: USER EDIT PRE-FLIGHT
    const double kMaximumTvcAngle = 7.5 * kDeg2Rad;
    const double kMaximumTvcAngleDeg = 7.5;
    const double kEngineMinThrust = 461.0; // N
    const double kEngineMaxThrust = 1107; // N
    const Eigen::Vector3d kEngineThrustLocationBodyM = Eigen::Vector3d(0.0, 0.0, -1.13);
    const double kControllerMinMomentArmM = 0.01;
    const double kControllerMinThrustForScalingN = 1.0;

    const double kControlIntegralPeriod = 0.25;
    const double kDeg2PulseWidth = ((double)1000.0) / ((double)90.0);
    const double kTvcXCenterAngleDeg = -10;
    const double kTvcYCenterAngleDeg = -55;
    const double kTvcYInputCenterAngleRad = 0.12;
    const double kTvcXInputCenterAngleRad = -0.29;
    const double TVCPeriod = 0.02;
    const int kTvcXPin = 19;
    const int kTvcYPin = 18;
    const int kNumberControllerGains = 10;
    const double weights_control_velocity = 0;
    const double weights_control_steady_state = 0;
    const double timeToStartControllerBeforeIgnite2 = 0.2;
    // Ignition constants, TODO: USER EDIT PRE-FlIGHT
    const int kIgnitionPin = 6;

    // Voltage reading, TODO: USER EDIT PRE-FLIGHT
    const double kR1 = 100000;
    const double kR2 = 10000;

    // Telemetry Constants
    const int BAUD_RATE = 9600;
    const int HARDWARE_SAVE_DELTA = 100;
    const int RF_SEND_DELTA = 300;

    // Time after launch until active stabalization begins.
    const float timeAtOffset = 0.0;

} // MissionConstants

#endif
