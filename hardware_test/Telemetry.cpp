#include <string>
#include <iostream>
#include <chrono>

#include "Mode.hpp"
#include "Telemetry.hpp"

void Telemetry::HardwareSaveFrame(Navigation& navigation, Controller& controller)
{
    // TODO write data to hardware file
    auto time_now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(time_now);

    // TODO Write time to file
    HardwareSaved << std::to_string(navigation.GetNavigation().coeff(0, 0)) << "\n";
}

void Telemetry::RfSendFrame(Navigation& navigation, Controller& controller)
{
    // TODO write data to rf file
    auto time_now = std::chrono::high_resolution_clock::now();

    // TODO Write time to file
    RFSent << std::to_string(navigation.GetNavigation().coeff(1, 0)) << "\n";
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