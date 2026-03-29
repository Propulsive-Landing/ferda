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
    const Eigen::Vector3d kEarthMagField = Eigen::Vector3d(-4.8415717072661559, 20.116087207173326, -46.952237491806379);
    const double kRad2Deg = 180 / kPi;
    const double kDeg2Rad = kPi / 180;

    // Structure constants
    const double kStructuresWetMassKg = 94.0;
    const double kStructuresDryMassKg = 66.0;
    const Eigen::Vector3d kStructuresWetCenterOfMassBodyM = Eigen::Vector3d(0.0, 0.0, 0.5);
    const Eigen::Vector3d kStructuresDryCenterOfMassBodyM = Eigen::Vector3d(0.0, 0.0, -0.5);
    const Eigen::Vector3d kStructuresWetMomentOfInertiaBodyKgm2 = Eigen::Vector3d(30.0, 30.0, 0.44);
    const Eigen::Vector3d kStructuresDryMomentOfInertiaBodyKgm2 = Eigen::Vector3d(20.0, 20.0, 0.34);
    
    // Navigation constants
    const Eigen::Vector3d kStructuresGroundOffset = Eigen::Vector3d(0.0, 0.0, -1.0); // Offset from marker data frame to ground frame
    const double kNavMagnetometerNoiseFactor = 1.0;
    const double kNavGPSPositionNoiseFactor = 1.0;
    const double kNavGPSVelocityNoiseFactor = 1.0;
    const double kNavLidarNoiseFactor = 10.0;
    const double kNavCameraNoiseFactor = 1.5;
    const double kNavAccelWhiteNoiseSigma = 0.0316;
    const double kNavGyroWhiteNoiseSigma = 0.00224;
    const double kNavAccelBiasRandomWalkSigma = 0.0;
    const double kNavGyroBiasRandomWalkSigma = 0.0;
    const double kNavPadVelocityNoiseMps = 1e-3;
    const double kNavPadAngularVelocityNoiseRadps = 0.00224;
    const double kNavInitialPositionVariance = 1e-5;
    const double kNavInitialVelocityVariance = 1e-6;
    const double kNavInitialAttitudeVariance = 1e-4;
    const double kNavInitialAccelBiasVariance = 1e-2;
    const double kNavInitialGyroBiasVariance = 1e-5;
    inline const Eigen::Matrix<double, 3, 3> kMarkerData =
        (Eigen::Matrix<double, 3, 3>() <<
            2.0, 2.0, 5.0,
            1.7320508075688774, -1.7320508075688774, 0.0,
            0.0,  0.0,  0.0
        ).finished();

    // Sensor constants, TODO: USER EDIT PRE-FLIGHT
    const double kSensorCameraNoise = 1e-3;
    const double kSensorMagnetometerNoise = 1; // Magnetometer measurement noise (Tesla)
    const double kSensorGPSPositionNoiseM = 3.0;
    const double kSensorGPSVelocityNoiseMps = 0.1;
    const double kSensorLidarNoiseM = 0.005;
    const Eigen::Vector3d kSensorCameraPosition = Eigen::Vector3d(0.2, 0.0, -1); // Position of the camera in the body frame (in meters)
    const Eigen::Vector3d kSensorCameraOrientationRad = Eigen::Vector3d(0.0, 2.3561944901923448, 0.0); // XYZ Euler orientation from camera frame to body frame
    const Eigen::Vector3d kSensorGPSPosition = Eigen::Vector3d(0.0, 0.0, 1.0); // Position of the GPS sensor (antenna) in the body frame (in meters)
    const Eigen::Vector3d kSensorMagnetometerPosition = Eigen::Vector3d(0.0, 0.0, 0.0); // Position of magnetometer in the body frame (in meters)

    // Throttle control constants, TODO: USER EDIT PRE-FLIGHT
    const double kMaximumTvcAngle = 7.5 * kDeg2Rad;
    const double kMaximumTvcAngleDeg = 7.5;
    const double kEngineMinThrust = 461.0; // N
    const double kEngineMaxThrust = 1107; // N
    const double kThrottleToMassFlowScale = -5.6e-4; // kg/(N*s)
    const Eigen::Vector3d kEngineThrustLocationBodyM = Eigen::Vector3d(0.0, 0.0, -1.13);
    const double kControllerMinMomentArmM = 0.01;
    const double kControllerMinThrustForScalingN = 1.0;

    // TVC Constants, TODO: USER EDIT PRE-FLIGHT
    const double kDeg2PulseWidth = ((double)1000.0) / ((double)90.0);
    const double kTvcXCenterAngleDeg = -10;
    const double kTvcYCenterAngleDeg = -55;
    const double kTvcYInputCenterAngleRad = 0.12;
    const double kTvcXInputCenterAngleRad = -0.29;
    const int kTvcXPin = 19;
    const int kTvcYPin = 18;
    
    // Ignition constants, TODO: USER EDIT PRE-FlIGHT
    const int kIgnitionPin = 6;

    // Voltage reading, TODO: USER EDIT PRE-FLIGHT
    const double kR1 = 100000;
    const double kR2 = 10000;

    // Telemetry Constants
    const int BAUD_RATE = 9600;
    const int HARDWARE_SAVE_DELTA = 100;
    const int RF_SEND_DELTA = 300;

} // MissionConstants

#endif
