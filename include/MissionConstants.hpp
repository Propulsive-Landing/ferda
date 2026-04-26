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
    const Eigen::Vector3d kStructuresGroundOffset = Eigen::Vector3d(0.0, 0.0, -0.74); // Position of the ground relative to the rocket's origin (in meters)

    // Navigation constants
    const double kNavMagnetometerNoiseFactor = 1.0;
    const double kNavGPSPositionNoiseFactor = 1.0;
    const double kNavGPSVelocityNoiseFactor = 1.0;
    const double kNavLidarNoiseFactor = 10.0;
    const double kNavCameraNoiseFactor = 1.5;
    const double kNavAccelWhiteNoiseSigma = 0.02;
    const double kNavGyroWhiteNoiseSigma = 0.0025;
    const double kNavAccelBiasRandomWalkSigma = 0.0;
    const double kNavGyroBiasRandomWalkSigma = 0.0;
    const double kNavCameraAssociationUnassignedPenaltyRad = 0.10;
    const double kNavInitialPositionVariance = 1e-5;
    const double kNavInitialVelocityVariance = 1e-6;
    const double kNavInitialAttitudeVariance = 1e-1;
    const double kNavInitialAccelBiasVariance = 0.5;
    const double kNavInitialGyroBiasVariance = 1e-6;
    inline const Eigen::Matrix<double, 3, 4> kMarkerData =
        (Eigen::Matrix<double, 3, 4>() << -0.042, 0.042, -0.042, 0.042,
         0.042, 0.042, -0.042, -0.042,
         0.0, 0.0, 0.0, 0.0)
            .finished();

    // Sensor constants, TODO: USER EDIT PRE-FLIGHT
    // Note that the intrinsic camera values are set to the default calibration values, but will be overridden if a valid calibration file is found at runtime. So they can be set to nominal values here for development and testing, and then updated with real calibration values once available.
    const double kSensorCameraNoise = 1e-3;
    const int kSensorCameraCaptureTimeoutMs = 1;
    const int kSensorCameraPreferredDeviceIndex = 0;
    const bool kSensorCameraUseGStreamer = true;
    const bool kSensorCameraUseCapAnyFallback = false;
    const bool kSensorCameraSaveDebugFrames = true;
    const std::string kSensorCameraDebugFrameDirectory = "../images";
    const std::string kSensorCameraCalibrationFilePath = "../calibration/camera_calibration.json";
    const double kSensorCameraFocalLengthXPx = 1450.0;
    const double kSensorCameraFocalLengthYPx = 1450.0;
    const double kSensorCameraPrincipalPointXPx = 960.0;
    const double kSensorCameraPrincipalPointYPx = 540.0;
    const double kSensorCameraDistortionK1 = 0.0;
    const double kSensorCameraDistortionK2 = 0.0;
    const double kSensorCameraDistortionP1 = 0.0;
    const double kSensorCameraDistortionP2 = 0.0;
    const double kSensorCameraDistortionK3 = 0.0;
    const int kSensorCameraImageWidthPx = 1920;
    const int kSensorCameraImageHeightPx = 1080;
    const int kSensorCameraMaxDetections = 5;
    const int kSensorCameraMarkerMinAreaPx = 2000;
    const double kSensorMagnetometerNoise = 1; // Magnetometer measurement noise (Tesla)
    const double kSensorGPSPositionNoiseM = 3.0;
    const double kSensorGPSVelocityNoiseMps = 0.1;
    const double kSensorLidarNoiseM = 0.005;
    const Eigen::Vector3d kSensorCameraPosition = Eigen::Vector3d(0.0, 0.0, 0.0);                     // Position of the camera in the body frame (in meters)
    const Eigen::Vector3d kSensorCameraOrientationRad = Eigen::Vector3d(3.141592653589793, 0.0, 0.0); // XYZ Euler orientation from camera frame to body frame
    const Eigen::Vector3d kSensorMagnetometerPosition = Eigen::Vector3d(0.0, 0.0, 0.0);               // Position of magnetometer in the body frame (in meters)
    // IMU axis remap from IMU sensor frame to vehicle body frame.
    // Must remain a right-angle transform: each row/column has exactly one +/-1 and zeros elsewhere.
    // Rows are body X/Y/Z, columns are sensor X/Y/Z.
    // Example swap and flip: body X = sensor Y, body Y = -sensor X, body Z = sensor Z
    // (Eigen::Matrix3i() << 0, 1, 0, -1, 0, 0, 0, 0, 1).finished();
    inline const Eigen::Matrix3i kSensorImuBodyAxisMap =
        (Eigen::Matrix3i() << 1, 0, 0,
         0, 0, -1,
         0, 1, 0)
            .finished();

    // Guidance constants, TODO: USER EDIT PRE-FLIGHT
    const double kGuidanceHoverDurationSeconds = 15.0;
    const double kGuidanceSlowReferenceVelocityMps = 0.5;
    const double kGuidanceFastReferenceVelocityMps = 5.0;
    const double kGuidanceHoverTargetAltitudeM = 10.0;
    const double kGuidanceTakeoffAltitudeThresholdM = 1.0;
    const double kGuidanceDescendTransitionAltitudeM = 2.0;
    const double kGuidanceAccelerationMargin = 0.75; // Use fraction of max acceleration/deceleration for safety margin

    // Throttle control constants, TODO: USER EDIT PRE-FLIGHT
    const double kMaximumTvcAngle = 10.0 * kDeg2Rad;
    const double kEngineMinThrust = 461.0;           // N
    const double kEngineMaxThrust = 1107;            // N
    const double kThrottleToMassFlowScale = -5.6e-4; // kg/(N*s)
    const Eigen::Vector3d kEngineThrustLocationBodyM = Eigen::Vector3d(0.0, 0.0, -1.13);

    // RCS yaw controller constants
    const double kControlRcsThrustCommand = 1.0;
    const double kControlRcsDeadbandRad = 2.0 * kDeg2Rad;
    const double kControlRcsDerivativeDeadbandRadPerSec = 2.0 * kDeg2Rad;

    // Shared PCA9685 constants
    inline const std::string PCA9685_I2C_ADDR = "/dev/i2c-1";

    // Servo Driver (PCA9685) Contants
    inline const float MAX_TICKS = 4095;
    inline const int SERVO_FREQ = 50;        // hz
    inline const float SERVO_PERIOD = 20000; // (us)
    inline const int SERVO_DRIVER_ADDR = 0x41;

    // 1000Hz PWM (PCA9685) constants
    inline const float PWM_FREQ = 1000;
    inline const float PWM_PERIOD = 0.01; // ms
    inline const int PWM_DRIVER_ADDR = 0x40;

    // ADS1115 constants
    const int ADS1ADDR = 0x48;
    const int ADS1BASE = 100;
    const int ADS2ADDR = 0x49;
    const int ADS2BASE = 200;
    const int ADS3ADDR = 0x4A;
    const int ADS3BASE = 300;

    // TVC Calibration Constants, TODO: USER EDIT PRE-FLIGHT
    const double kTvcYInputCenterAngleRad = 0.0;
    const double kTvcXInputCenterAngleRad = 0.0;
    
    // Linear Actuator TVC Geometry (in inches, relative to u-joint origin)
    const Eigen::Vector3d kTvcVehicleMountPoint0 = Eigen::Vector3d(5.688, 0.0, 6.408);
    const Eigen::Vector3d kTvcVehicleMountPoint1 = Eigen::Vector3d(0, 5.688, 6.408);
    const Eigen::Vector3d kTvcEngineMountPoint0 = Eigen::Vector3d(2.985, 0.0, -11.078);
    const Eigen::Vector3d kTvcEngineMountPoint1 = Eigen::Vector3d(0.0, 2.985, -11.078);

    // Linear Actuator Control Parameters
    // PID output is commanded actuator speed in inches/second.
    const double kTvcPositionKpPerSecond = 5.0;                 // (in/s)/in = 1/s
    const double kTvcPositionKiPerSecondSquared = 0.0;          // (in/s)/(in*s) = 1/s^2
    const double kTvcPositionKdUnitless = 0.0;                 // (in/s)/(in/s) = unitless
    const double kTvcIntegralWindupLimitInchSeconds = 1.0;      // Clamp for integrated position error
    const double kTvcVelocityDeadbandInchesPerSecond = 0.01;    // Velocity deadband to avoid chatter
    const double kTvcMaxCommandedVelocityInchesPerSecond = 1.8; // Maps to max PWM command
    const int kTvcMaxMotorSpeed = 3686;                         // Max PWM speed (0-4095)
    const double kTvcStrokeLengthInches = 4.0;                  // Actuator stroke length
    const double kTvcZeroExtensionInches = 14.972;              // Actuator length when fully retracted (used to convert from absolute length to extension length)
    const double kTvcMinLengthInches = 2.0;                     // Minimum actuator extension
    const double kTvcMaxLengthInches = 3.9;                    // Maximum actuator extension
    const int kTvcActuator0PotentiometerMinReading = 26054;     // ADC reading for actuator 0 at minimum extension (retracted)
    const int kTvcActuator0PotentiometerMaxReading = 2498;      // ADC reading for actuator 0 at maximum extension (extended)
    const int kTvcActuator1PotentiometerMinReading = 26054;     // ADC reading for actuator 1 at minimum extension (retracted)
    const int kTvcActuator1PotentiometerMaxReading = 2498;      // ADC reading for actuator 1 at maximum extension (extended)
    const int kTvcActuator0RpwmChannel = 0;                     // PCA9685 RPWM channel for actuator 0 (X)
    const int kTvcActuator0LpwmChannel = 1;                     // PCA9685 LPWM channel for actuator 0 (X)
    const int kTvcActuator1RpwmChannel = 2;                     // PCA9685 RPWM channel for actuator 1 (Y)
    const int kTvcActuator1LpwmChannel = 3;                     // PCA9685 LPWM channel for actuator 1 (Y)
    const double kTvcCommandTimeoutSeconds = 0.100;             // Max command age before actuator output is forced to neutral
    const float kTvcChirpDurationSec = 10.0f;
    const float kTvcChirpStartFreqHz = 1.0f;
    const float kTvcChirpEndFreqHz = 15.0f;
    const int kTvcChirpMaxSpeed = kTvcMaxMotorSpeed;
    const int kTvcChirpControlPeriodMs = 20;
    const int kTVCXPotentiometerReading = ADS3BASE + 0; // Third ADS1115 A0
    const int kTVCYPotentiometerReading = ADS3BASE + 1; // Third ADS1115 A1

    // Ignition constants, TODO: USER EDIT PRE-FlIGHT
    const int kIgnitionPin = 6;

    // Liquid Propulsion Hardware Pins, TODO: USER EDIT PRE-FLIGHT
    // Valve Servo Pins (PWM)
    const int kNitrogenServoPin = 6;     // servo driver row [0-15]
    const int kPurgeServoPin = 0;        // servo driver row [0-15]
    const int kMainEthanolServoPin = 11; // servo driver row [0-15]
    const int kMainNitrousServoPin = 0;  // servo driver row [0-15]
    const int kNitrousFillServoPin = 5;  // servo driver row [0-15]

    // Valve Solenoid Pins (GPIO)
    const int kASIEthanolPin = 0;
    const int kASIOxygenPin = 5;
    const int kNitrogenBleedPin = 0;

    // Spark Plug Pins
    const int kSparkPin = 0; // Relay control
    const int kRPMPin = 0;   // PWM output // servo driver row [0-15]

    // Pressure Transducer Pins (Analog)
    const int kNitrogenLinePTPin = ADS1BASE + 0;    // First ADS1115 A0
    const int kEthanolTankPTPin = ADS1BASE + 1;     // First ADS1115  A1
    const int kNitrousLinePTPin = ADS1BASE + 2;     // First ADS1115 A2
    const int kOxygenLinePTPin = ADS1BASE + 3;      // First ADS1115  A3
    const int kFuelInletPTPin = ADS2BASE + 0;       // Second ADS1115 A0
    const int kFuelOutletPTPin = ADS2BASE + 1;      // Second ADS1115 A1
    const int kChamberPressurePTPin = ADS2BASE + 2; // Second ADS1115 A2

    // Load Cell Pin (Analog)
    const int kLoadCellPin = ADS2BASE + 3; // Second ADS1115 A3

    // Valve Servo Angles
    const int kValveClosedAngle = 179; // degrees
    const int kValveOpenAngle = 91;    // degrees

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
    const int IMU_I2C_ADDR = 0x28;
    const int CHIP_ID_ADDR = 0x00;
    const int CHIP_ID = 0xA0;
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
    inline const char *RF_Port = "/dev/ttyUSB0";

    // GPS constants
    const int MAX_SIZE = 1000;
    inline const char *GPS_Port = "/dev/ttyUSB1";

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
