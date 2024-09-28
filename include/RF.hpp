#include <string>
#include <fstream>
#include <stdio.h>


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
            Ignite,
            Release
        };


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
