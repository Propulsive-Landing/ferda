#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

#ifndef __MISSION_CONSTANTS__

#define __MISSION_CONSTANTS__


namespace MissionConstants {
    // YAML::Node LoadConstants(std::string filepath) {
    //     YAML::Node constants = YAML::LoadFile(filepath);
    //     return constants;
    // }
    
    // Physical constants
    const double kPi = 3.1415926535897932384626433;
    const double kGravity = 9.80298; //calculated at Ashford Town Park using https://www.sensorsone.com/local-gravity-calculator/#height
    const double kPressureH = 0.0; //TODO: calculate H
    const double kRad2Deg = 180/kPi;
    const double kDeg2Rad = kPi/180;

    // Navigation constants
    const double kNavThetaDotSmooth = 0.05;
    const double kFswLoopTime = .005;
    const double kFSWCalibrationTime = 0.05;

    // Controller constants, TODO: USER EDIT PRE-FLIGHT
    const double kMaximumTvcAngle = 7.5*kDeg2Rad;
    const double kControlIntegralPeriod = 0.25;
    const double kDeg2PulseWidth = ((double) 1000.0)/((double) 90.0);
    const double kTvcXCenterAngle = -10;
    const double kTvcYCenterAngle = -55;
    const double TVCPeriod = 0.02;
    const int kTvcXPin = 19;
    const int kTvcYPin = 18;
    const std::string kKMatrixFile = "k_matrix.csv";
    const int kNumberControllerGains = 10;
    
    // Ignition constants, TODO: USER EDIT PRE-FlIGHT
    const int kIgnitionPin = 6;
    
    // Voltage reading, TODO: USER EDIT PRE-FLIGHT
    const double kR1 = 100000;
    const double kR2 = 10000;

    // Telemetry Constants
    const int BAUD_RATE = 9600;
    const int HARDWARE_SAVE_DELTA = 100;
    const int RF_SEND_DELTA = 300;

    // Abort constants

    // Time after launch until active stabalization begins.
    const float timeAtOffset = 0.0;

} // MissionConstants

#endif