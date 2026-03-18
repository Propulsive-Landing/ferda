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
    const double kPressureH = 0.0;   // TODO: calculate H
    const double kRad2Deg = 180 / kPi;
    const double kDeg2Rad = kPi / 180;

    // Navigation constants
    const double kNavThetaDotSmooth = 0.05;
    const double kFswLoopTime = .005;
    const double kFSWCalibrationTime = 0.05;
    const double originalOffsetAngle = 5 * kDeg2Rad;
    const Eigen::Vector3d kEarthMagField = Eigen::Vector3d(-0.089, 0.378, -0.921).normalized(); // Unit vector pointing in the direction of Earth's magnetic field
    const Eigen::Vector3d kSensorCameraPosition = Eigen::Vector3d(0.2, 0.2, -1);                // Position of the camera in the body frame (in meters)
    inline const Eigen::Matrix<double, 3, 3> kMarkerData =
        (Eigen::Matrix<double, 3, 3>() << -2.5, -2.5, 5.0,
         4.3301, -4.3301, 0.0,
         0.5, 0.5, 0.5)
            .finished();

    // Controller constants, TODO: USER EDIT PRE-FLIGHT
    const double kMaximumTvcAngle = 7.5 * kDeg2Rad;
    const double kMaximumTvcAngleDeg = 7.5;

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
    const int MAX_SIZE = 256;
    inline const char *GPS_Port = "/dev/ttyUSB0";

    //inline const char *GPS_Port = "/dev/cu.usbserial-110";

    namespace NMEA
    {
        const int MESSAGE_TYPE_STARTING_STRING_INDEX = 3;
        const std::unordered_set<std::string> NMEA_MESSAGE_TYPE_SET = {std::string("RMC"), std::string("GGA")};
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
