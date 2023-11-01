#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <stdio.h>
#include <sys/poll.h> 

#include "Mode.hpp"
#include "Telemetry.hpp"

void Telemetry::HardwareSaveFrame(Navigation& navigation, Controller& controller)
{
    // write time to hardware file
    auto time_now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(time_now);

    HardwareSaved << std::put_time(std::localtime(&in_time_t), "%c") << ",";

    // Write data to file
    HardwareSaved << std::to_string(navigation.GetNavigation().coeff(0, 0)) << "\n" << std::flush;
}

void Telemetry::RfSendFrame(Navigation& navigation, Controller& controller)
{
    // write data to rf file
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    // Print data to stdout
    std::cout << std::to_string(navigation.GetNavigation().coeff(1, 0));

     // Write time to file
    RFSent << std::put_time(std::localtime(&in_time_t), "%c") << ",";
    
    //Write data to file
    RFSent << std::to_string(navigation.GetNavigation().coeff(1, 0)) << "\n" << std::flush;;
}


void Telemetry::SendString(std::string message) {
    // Add time tag to file
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::cout << message << "\n";

    // write time to file
    this->Logs << std::put_time(std::localtime(&in_time_t), "%c") << ",";
    this->Logs << message << "\n" << std::flush;
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

Mode::Phase Telemetry::CheckAndHandleCommand() {
    struct pollfd fds;
    int ret;
    fds.fd = 0; /* this is STDIN */
    fds.events = POLLIN;
    ret = poll(&fds, 1, 0);

    if(ret != 1) // Return if no data
        return;

    std::string input_line;
    getline(std::cin, input_line);
    this->Logs << "GOT: " << input_line << "\n" << std::flush;
}

Telemetry::Telemetry()
{
    Logs.open ("../logs/logs.txt");
    RFSent.open ("../logs/RFlogs.txt");;
    HardwareSaved.open ("../logs/data.txt");;

    //TODO Write headers to data file where needed
}

Telemetry::~Telemetry()
{
    Logs.close();
    RFSent.close();
    HardwareSaved.close();
}