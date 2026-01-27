#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

#ifndef __MISSION_CONSTANTS__

#define __MISSION_CONSTANTS__

namespace MissionConstants
{
    // YAML::Node LoadConstants(std::string filepath) {
    //     YAML::Node constants = YAML::LoadFile(filepath);
    //     return constants;
    // }

    const bool isStabilityTest = false;

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

    // Liquid Propulsion Hardware Pins, TODO: USER EDIT PRE-FLIGHT
    // Valve Servo Pins (PWM)
    const int kNitrogenServoPin = 6;
    const int kPurgeServoPin = 0;
    const int kMainEthanolServoPin = 11;
    const int kMainNitrousServoPin = 0;

    // Valve Solenoid Pins (GPIO)
    const int kASIEthanolPin = 0;
    const int kASIOxygenPin = 5;
    const int kNitrogenBleedPin = 0;

    // Spark Plug Pins
    const int kSparkPin = 0; // Relay control
    const int kRPMPin = 0;   // PWM output

    // Pressure Transducer Pins (Analog)
    const int kNitrogenLinePTPin = 0;    // A0
    const int kEthanolTankPTPin = 0;     // A0
    const int kNitrousLinePTPin = 0;     // A0
    const int kOxygenLinePTPin = 0;      // A0
    const int kFuelInletPTPin = 0;       // A0
    const int kFuelOutletPTPin = 0;      // A0
    const int kChamberPressurePTPin = 0; // A0

    // Load Cell Pin (Analog)
    const int kLoadCellPin = 0; // A0

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

    // Mode constants
    const double motor_thrust_duration = 2.09;
    const double motor_thrust_percentage = 1;
    const double gse_height = 0.2800;
    const double second_motor_delta_x = 25.96;

} // MissionConstants

#endif
