#include <string>
#include <fstream>
#include <stdio.h>
#include <map>
#include <regex>

#define FRAME_MAGIC_NUMBER 0xDEADBEEF  // signals a new data frame, used in RF transmission
#define STRING_MAGIC_NUMBER 0xBABAFACE // Signals a new string, used in RF transmission
#define LOG_CHAR_LENGTH 256
#define RF_FOOTER 0xCAFEFADE // Footer to validate RF data, used in RF transmission


class RF {
    private:
        RF();
        ~RF();

        int SerialFd; // Not used in testing class

    public:
        
        enum Command {
            None,
            ABORT,
            Startup,
            TestTVC,
            GoIdle,
            Ignite,
            Release,
            IncrementXTVC,
            IncrementYTVC,
            DecrementXTVC,
            DecrementYTVC,
        };


        ParseCommand(std::string input_line){        
            // Define the regular expression to match leading and trailing whitespace/newline characters
            std::regex pattern("^\\s+|\\s+$");

            // Replace leading and trailing whitespace/newline characters with an empty string
            input_line = std::regex_replace(input_line, pattern, "");

            RF::Command ParsedCommand = RF::Command::None;
            if(input_line == "ABORT")
                ParsedCommand = RF::Command::ABORT;
            else if(input_line == "Startup")
                ParsedCommand = RF::Command::Startup;
            else if(input_line == "TestTVC")
                ParsedCommand = RF::Command::TestTVC;
            else if(input_line == "GoIdle")
                ParsedCommand = RF::Command::GoIdle;
            else if(input_line == "Ignite")
                ParsedCommand = RF::Command::Ignite;
            else if(input_line == "IncrementYTVC")
                ParsedCommand = RF::Command::IncrementYTVC;
            else if(input_line == "IncrementXTVC")
                ParsedCommand = RF::Command::IncrementXTVC;
            else if(input_line == "DecrementXTVC")
                ParsedCommand = RF::Command::DecrementXTVC;
            else if(input_line == "DecrementYTVC")
                ParsedCommand = RF::Command::DecrementYTVC;
            else if(input_line == "Release")
                ParsedCommand = RF::Command::Release;
            else
                ParsedCommand = RF::Command::None;

            return ParsedCommand;
        }

        struct rfFrame // A structure derrived from telemFrame that only contains a subset of attributes in order to save space.
        {
            uint32_t magic_number;

            uint16_t mode;
            float euler[3];
            float velocity[3];
	        float input[2];
            float dt;

            uint32_t footer;
        } __attribute__((packed));

        

        std::ofstream RFSent;

        void SendString(std::string message);
        void SendFrame(RF::rfFrame frame);

        static RF& GetInstance()
        {
            static RF rf;

            return rf;
        }


        RF::Command GetCommand(); // Will check for commands and return the received command. Non-blocking.

};
