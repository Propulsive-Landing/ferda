#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <iostream>
#include <Eigen/Dense>

#ifndef __MISSION_CONSTANTS__

#define __MISSION_CONSTANTS__

// TODO: add more namespaces to sub types like navigation and controller, etc
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
    const Eigen::Vector3d kStructuresGroundOffset = Eigen::Vector3d(0.0, 0.0, -1.0); // Position of the ground relative to the rocket's origin (in meters)
    
    // Navigation constants
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
    const double kNavInitialAttitudeVariance = 1e-2;
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

    // Guidance constants, TODO: USER EDIT PRE-FLIGHT
    const double kGuidanceHoverDurationSeconds = 15.0;
    const double kGuidanceSlowReferenceVelocityMps = 0.5;
    const double kGuidanceFastReferenceVelocityMps = 5.0;
    const double kGuidanceHoverTargetAltitudeM = 10.0;
    const double kGuidanceTakeoffAltitudeThresholdM = 1.0;
    const double kGuidanceDescendTransitionAltitudeM = 2.0;
    const double kGuidanceAccelerationMargin = 0.75; // Use fraction of max acceleration/deceleration for safety margin

    // Throttle control constants, TODO: USER EDIT PRE-FLIGHT
    const double kMaximumTvcAngle = 7.5 * kDeg2Rad;
    const double kMaximumTvcAngleDeg = 7.5;
    const double kEngineMinThrust = 461.0; // N
    const double kEngineMaxThrust = 1107; // N
    const double kThrottleToMassFlowScale = -5.6e-4; // kg/(N*s)
    const Eigen::Vector3d kEngineThrustLocationBodyM = Eigen::Vector3d(0.0, 0.0, -1.13);

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

    // Time after launch until active stabalization begins.
    const float timeAtOffset = 0.0;

    // IMU constants
    const int IMU_i2c_addr = 0x28;
    const int POWER_MODE = 0x3E;
    const int POWER_NORMAL = 0x00;
    const int OPERATION_MODE = 0x3D;
    const int CONFIG = 0x00;
    const int AMG = 0x07;
    const int REG_ACC_X = 0x08;
    const int REG_ACC_Y = 0x0A;
    const int REG_ACC_Z = 0x0C;
    const int REG_GYRO_X = 0x14;
    const int REG_GYRO_Y = 0x16;
    const int REG_GYRO_Z = 0x18;
    const int REG_MAG_X = 0x0E;
    const int REG_MAG_Y = 0x10;
    const int REG_MAG_Z = 0x12;

    const int UNIT_SEL = 0X3B;
    const int RAD = 0x02;

    // RF consants
    inline const char *RF_Port = "/dev/ttyS0";

    // GPS constants
    const int MAX_SIZE = 1000;
    inline const char *GPS_Port = "/dev/ttyUSB0";

    //inline const char *GPS_Port = "/dev/cu.usbserial-110";

    namespace NMEA
    {
        const int MESSAGE_TYPE_STARTING_STRING_INDEX = 3;
        const int MESSAGE_TYPE_IDX = 0; // 0 index BASED
        const int TIME_IDX = 1;         // 0 index BASED
        namespace RMC
        {
            const std::string RMC = "RMC"; // for latutude, longitude, speed (knots), time
            const int NUM_VALUES = 13;
            const int STATUS_IDX = 2;              // Starting from 0 indexed
            const char BAD_STATUS_CHARACTER = 'V'; // Since GGA can have 2 validity indicators and 1 invalid inidactor, it's
                                                   // easier if all NMEA ouputs check for invalidty
            const int LATITUDE_IDX = 3;            // 0 index based
            const int LATITUDE_DIRECTION_IDX = 4;
            const int LONGITUDE_IDX = 5; // 0 index based
            const int LONGITUDE_DIRECTION_IDX = 6;
            const int COURSE_IDX = 8;
            const int SPEED_IDX = 7; // 0 based also in knots
            inline float time;
        };
        namespace GGA
        {

            const std::string GGA = "GGA"; // for altitude, time
            const int NUM_VALUES = 15;
            const int STATUS_IDX = 7;        // Starting from 1 indexed
            const int BAD_STATUS_NUMBER = 0; // Since there are 2 valid indicators and 1 invalid, it's easier to check
                                             // for invalid
            const int ALTITUDE_INDEX = 9;    // 0 based

            inline float time;
        };
    };

}; // MissionConstants

#endif
