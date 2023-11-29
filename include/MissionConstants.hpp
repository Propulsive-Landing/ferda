#pragma once

#include <string>

namespace MissionConstants {
    // YAML::Node LoadConstants(std::string filepath) {
    //     YAML::Node constants = YAML::LoadFile(filepath);
    //     return constants;
    // }
    
    // Physical constants
    const float kPi = 3.1415926535897932384626433;
    const float kGravity = 9.80298; //calculated at Ashford Town Park using https://www.sensorsone.com/local-gravity-calculator/#height
    const float kPressureH = 0.0; //TODO: calculate H
    const float kRad2Deg = 180/kPi;
    const float kDeg2Rad = kPi/180;

    // Navigation constants
    const float kNavThetaDot_smooth

    // Controller constants, TODO: USER EDIT PRE-FLIGHT
    const float kMaximumTvcAngle = 7.5*kDeg2Rad;
    const float kControlIntegralPeriod = 0.25;
    const float kDeg2PulseWidth = ((float) 1000.0)/((float) 90.0);
    const float kTvcXCenterPulseWidth = 1529;
    const float kTvcYCenterPulseWidth = 915;
    const int kTvcXPin = 19;
    const int kTvcYPin = 18;
    const std::string kKMatrixFile = "k_matrix.csv";
    const int kNumberControllerGains = 10;
    
    // Ignition constants, TODO: USER EDIT PRE-FlIGHT
    const int kIgnitionPin = 6;
    
    // Voltage reading, TODO: USER EDIT PRE-FLIGHT
    const float kR1 = 100000;
    const float kR2 = 10000;

    // Telemetry Constants
    const int BAUD_RATE = 9600;
    const int HARDWARE_SAVE_DELTA = 100;
    const int RF_SEND_DELTA = 300;

    // Abort constants

} // MissionConstants