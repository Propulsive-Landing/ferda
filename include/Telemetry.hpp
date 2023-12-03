#include <string>
#include <fstream>
#include <stdio.h>

#include "Mode.hpp"
#include "Navigation.hpp"

class Telemetry {
    private:
        std::ofstream Logs;
        std::ofstream RFSent;
        std::ofstream HardwareSaved;

	FILE *SerialPort;

        Telemetry();
        ~Telemetry();

        struct rfFrame // A structure derrived from telemFrame that only contains a subset of attributes in order to save space.
        {
            uint32_t magic_number;

            uint16_t mode;
            float euler[3];
            float velocity[3];
            float dt;

            uint32_t footer;
        } __attribute__((packed));

        void HardwareSaveFrame(Navigation& navigation, Controller& controller);
        void RfSendFrame(Navigation& navigation, Controller& controller);

    public:
        enum Command {
            None,
            ABORT,
            Startup,
            Ignite,
            Release
        };
        void RunTelemetry(Navigation& navigation, Controller& controller, float HardwareSaveDelta, float RFSendDelta);
        void SendString(std::string message); // Send string over RF
        void SendCommand(Command command);
        void SendData(Navigation &navigation, Controller &controller); // Send navigation data over RF
        Telemetry::Command GetCommand(); // Will check for commands and return the received command. Non-blocking.


        static Telemetry& GetInstance()
        {
            static Telemetry telem;

            return telem;
        }

};
