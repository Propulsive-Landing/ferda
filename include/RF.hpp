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
        Startup,
        TestTVC,
        ChirpTVC,
        GoIdle,
        Ignite,
        Release,
        ActuatorCalibration,
        StopTVC,
        CenterTVC,
        MoveXTVCToLimitExtend,
        MoveXTVCToLimitRetract,
        MoveYTVCToLimitExtend,
        MoveYTVCToLimitRetract,
        IncrementXTVC,
        IncrementYTVC,
        DecrementXTVC,
        DecrementYTVC
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
        else if (input_line == "Startup")
            ParsedCommand = RF::Command::Startup;
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
        else if (input_line == "Release")
            ParsedCommand = RF::Command::Release;
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
