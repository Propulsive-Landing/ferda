#include <string>
#include <iostream>
#include <chrono>

#include "Mode.hpp"
#include "Telemetry.hpp"

void HardwareSaveFrame(Navigation& navigation, Controller& controller)
{
    // TODO write data to hardware file
}

void RfSendFrame(Navigation& navigation, Controller& controller)
{
    // TODO write data to rf file
}


void Telemetry::SendString(std::string message) {

    // TODO write log to file
    std::cout << message;
    this->Logs << message << "\n";
}

void Telemetry::RunTelemetry(Navigation& navigation, Controller& controller, float HardwareSaveDelta, float RFSendDelta) {
        /* Start calculate time change*/
        static auto last_time = std::chrono::high_resolution_clock::now();
        auto time_now = std::chrono::high_resolution_clock::now();
        double change_time = (time_now.time_since_epoch() - last_time.time_since_epoch()).count();
        /* End calculate time change*/

        if(change_time >= HardwareSaveDelta){
            HardwareSaveFrame(navigation, controller);
            last_time = std::chrono::high_resolution_clock::now();
        }
        if(change_time >= RFSendDelta){
            RfSendFrame(navigation, controller);
            last_time = std::chrono::high_resolution_clock::now();
        }
}

void Telemetry::CheckAndHandleCommand(Mode::Phase &eCurrentMode) {
    
}

Telemetry::Telemetry()
{
    Logs.open ("logs.txt");
    RFSent.open ("RFlogs.txt");;
    HardwareSaved.open ("data.txt");;
}

Telemetry::~Telemetry()
{
    Logs.close();
    RFSent.close();
    HardwareSaved.close();
}