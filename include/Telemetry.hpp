#include <string>
#include <fstream>

#include "Navigation.hpp"

class Telemetry {

    std::ofstream Logs;
    std::ofstream RFSent;
    std::ofstream HardwareSaved;

    struct rfFrame // A structure derrived from telemFrame that only contains a subset of attributes in order to save space.
    {
        uint32_t magic_number;

        uint16_t mode;
        float euler[3];
        float velocity[3];
        float dt;

        uint32_t footer;
    } __attribute__((packed));

    void RunTelemetry(Navigation& navigation, Controller& controller, float HardwareSaveDelta, float RFSendDelta);
    void SendString(std::string message); // Send string over RF
    void SendData(Navigation &navigation, Controller &controller); // Send navigation data over RF
    void CheckAndHandleCommand(Mode::Phase &eCurrentMode); // Will check for commands and modify mode if needed
    Telemetry();
    ~Telemetry();

    static Telemetry& GetInstance()
    {
        static Telemetry telem;

        return telem;
    } 

};
