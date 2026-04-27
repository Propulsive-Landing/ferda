#pragma once

#include <string>
#include <fstream>
#include <stdio.h>
#include <map>
#include <regex>

#define FRAME_MAGIC_NUMBER 0xDEADBEEF  // signals a new data frame, used in RF transmission
#define STRING_MAGIC_NUMBER 0xBABAFACE // Signals a new string, used in RF transmission
#define LOG_CHAR_LENGTH 256
#define RF_FOOTER 0xCAFEFADE // Footer to validate RF data, used in RF transmission

class RF
{
private:
    RF();
    ~RF();

    int SerialFd;                 // Not used in testing class
    bool terminal_switch = false; // Used to determine how to communicate with Flight Computer (If XBEE fails, switch to terminal)

public:
    enum Command
    {
        None,
        ABORT,
        ABORT_PAD,
        ABORT_GROUND,
        TestTVC,
        ChirpTVC,
        GoIdle,
        Ignite,
        ActuatorCalibration,
        StopTVC,
        CenterTVC,
        NAV_RESTART,
        MoveXTVCToLimitExtend,
        MoveXTVCToLimitRetract,
        MoveYTVCToLimitExtend,
        MoveYTVCToLimitRetract,
        IncrementXTVC,
        IncrementYTVC,
        DecrementXTVC,
        DecrementYTVC,
        AccelBias,
        GyroBias,
        // Liquid Propulsion Commands
        ValveNitrogenOpen,
        ValveNitrogenClose,
        ValvePurgeOpen,
        ValvePurgeClose,
        ValveMainEthanolOpen,
        ValveMainEthanolClose,
        ValveMainNitrousOpen,
        ValveMainNitrousClose,
        ValveNitrousFillOpen,
        ValveNitrousFillClose,
        ValveASIEthanolOpen,
        ValveASIEthanolClose,
        ValveASIOxygenOpen,
        ValveASIOxygenClose,
        ValveNitrogenBleedOpen,
        ValveNitrogenBleedClose,
        SparkOn,
        SparkOff,
        ASITest,
        WaterFlow,
        ThreeSecondHotfire,
        GoHotfireIdle,
        CameraOn,
        CameraOff,
        LidarOn,
        LidarOff,
        GPSVelocityOn,
        GPSVelocityOff,
        GPSPositionOn,
        GPSPositionOff,
        MagnetometerOn,
        MagnetometerOff,
    };

    RF::Command ParseCommand(std::string input_line)
    {
        // Define the regular expression to match leading and trailing whitespace/newline characters
        std::regex pattern("^\\s+|\\s+$");

        // Replace leading and trailing whitespace/newline characters with an empty string
        input_line = std::regex_replace(input_line, pattern, "");

        RF::Command ParsedCommand = RF::Command::None;
        if (input_line == "ABORT")
            ParsedCommand = RF::Command::ABORT;
        else if (input_line == "ABORT_PAD")
            ParsedCommand = RF::Command::ABORT_PAD;
        else if (input_line == "ABORT_GROUND")
            ParsedCommand = RF::Command::ABORT_GROUND;
        else if (input_line == "TestTVC")
            ParsedCommand = RF::Command::TestTVC;
        else if (input_line == "ChirpTVC")
            ParsedCommand = RF::Command::ChirpTVC;
        else if (input_line == "GoIdle")
            ParsedCommand = RF::Command::GoIdle;
        else if (input_line == "Ignite")
            ParsedCommand = RF::Command::Ignite;
        else if (input_line == "ActuatorCalibration" || input_line == "ActuatorCalibrate" || input_line == "ACTUATOR_CALIBRATION")
            ParsedCommand = RF::Command::ActuatorCalibration;
        else if (input_line == "StopTVC" || input_line == "STOP" || input_line == "stop")
            ParsedCommand = RF::Command::StopTVC;
        else if (input_line == "CenterTVC" || input_line == "CENTER" || input_line == "center")
            ParsedCommand = RF::Command::CenterTVC;
        else if (input_line == "NAV_RESTART")
            ParsedCommand = RF::Command::NAV_RESTART;
        else if (input_line == "MoveXTVCToLimitExtend" || input_line == "MoveXToLimitExtend" || input_line == "MOVE_X_TO_LIMIT")
            ParsedCommand = RF::Command::MoveXTVCToLimitExtend;
        else if (input_line == "MoveXTVCToLimitRetract" || input_line == "MoveXToLimitRetract")
            ParsedCommand = RF::Command::MoveXTVCToLimitRetract;
        else if (input_line == "MoveYTVCToLimitExtend" || input_line == "MoveYToLimitExtend" || input_line == "MOVE_Y_TO_LIMIT")
            ParsedCommand = RF::Command::MoveYTVCToLimitExtend;
        else if (input_line == "MoveYTVCToLimitRetract" || input_line == "MoveYToLimitRetract")
            ParsedCommand = RF::Command::MoveYTVCToLimitRetract;
        else if (input_line == "IncrementYTVC")
            ParsedCommand = RF::Command::IncrementYTVC;
        else if (input_line == "IncrementXTVC")
            ParsedCommand = RF::Command::IncrementXTVC;
        else if (input_line == "DecrementXTVC")
            ParsedCommand = RF::Command::DecrementXTVC;
        else if (input_line == "DecrementYTVC")
            ParsedCommand = RF::Command::DecrementYTVC;
        else if (input_line == "SENSOR: camera ON" || input_line == "CameraOn")
            ParsedCommand = RF::Command::CameraOn;
        else if (input_line == "SENSOR: camera OFF" || input_line == "CameraOff")
            ParsedCommand = RF::Command::CameraOff;
        else if (input_line == "SENSOR: lidar ON")
            ParsedCommand = RF::Command::LidarOn;
        else if (input_line == "SENSOR: lidar OFF")
            ParsedCommand = RF::Command::LidarOff;
        else if (input_line == "SENSOR: gps_velocity ON")
            ParsedCommand = RF::Command::GPSVelocityOn;
        else if (input_line == "SENSOR: gps_velocity OFF")
            ParsedCommand = RF::Command::GPSVelocityOff;
        else if (input_line == "SENSOR: gps_position ON")
            ParsedCommand = RF::Command::GPSPositionOn;
        else if (input_line == "SENSOR: gps_position OFF")
            ParsedCommand = RF::Command::GPSPositionOff;
        else if (input_line == "SENSOR: magnetometer ON")
            ParsedCommand = RF::Command::MagnetometerOn;
        else if (input_line == "SENSOR: magnetometer OFF")
            ParsedCommand = RF::Command::MagnetometerOff;
        // Liquid Propulsion Commands
        else if (input_line == "VALVE: nitrogen open")
            ParsedCommand = RF::Command::ValveNitrogenOpen;
        else if (input_line == "VALVE: nitrogen close")
            ParsedCommand = RF::Command::ValveNitrogenClose;
        else if (input_line == "VALVE: purge open")
            ParsedCommand = RF::Command::ValvePurgeOpen;
        else if (input_line == "VALVE: purge close")
            ParsedCommand = RF::Command::ValvePurgeClose;
        else if (input_line == "VALVE: main ethanol open")
            ParsedCommand = RF::Command::ValveMainEthanolOpen;
        else if (input_line == "VALVE: main ethanol close")
            ParsedCommand = RF::Command::ValveMainEthanolClose;
        else if (input_line == "VALVE: main nitrous open")
            ParsedCommand = RF::Command::ValveMainNitrousOpen;
        else if (input_line == "VALVE: main nitrous close")
            ParsedCommand = RF::Command::ValveMainNitrousClose;
        else if (input_line == "VALVE: nitrous fill open")
            ParsedCommand = RF::Command::ValveNitrousFillOpen;
        else if (input_line == "VALVE: nitrous fill close")
            ParsedCommand = RF::Command::ValveNitrousFillClose;
        else if (input_line == "VALVE: ASI ethanol open")
            ParsedCommand = RF::Command::ValveASIEthanolOpen;
        else if (input_line == "VALVE: ASI ethanol close")
            ParsedCommand = RF::Command::ValveASIEthanolClose;
        else if (input_line == "VALVE: ASI oxygen open")
            ParsedCommand = RF::Command::ValveASIOxygenOpen;
        else if (input_line == "VALVE: ASI oxygen close")
            ParsedCommand = RF::Command::ValveASIOxygenClose;
        else if (input_line == "VALVE: nitrogen bleed open")
            ParsedCommand = RF::Command::ValveNitrogenBleedOpen;
        else if (input_line == "VALVE: nitrogen bleed close")
            ParsedCommand = RF::Command::ValveNitrogenBleedClose;
        else if (input_line == "SPARK: on")
            ParsedCommand = RF::Command::SparkOn;
        else if (input_line == "SPARK: off")
            ParsedCommand = RF::Command::SparkOff;
        else if (input_line == "asitest")
            ParsedCommand = RF::Command::ASITest;
        else if (input_line == "waterflow")
            ParsedCommand = RF::Command::WaterFlow;
        else if (input_line == "3second")
            ParsedCommand = RF::Command::ThreeSecondHotfire;
        else if (input_line == "GoHotfireIdle")
            ParsedCommand = RF::Command::GoHotfireIdle;
        else
            ParsedCommand = RF::Command::None;

        return ParsedCommand;
    }

    std::ofstream RFSent;

    void SendString(std::string message);

    static RF &GetInstance()
    {
        static RF rf;

        return rf;
    }

    RF::Command GetCommand(); // Will check for commands and return the received command. Non-blocking.
};