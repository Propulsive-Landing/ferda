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


void Telemetry::Log(std::string message) {
    // write time to hardware file
    auto time_now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(time_now);

    Logs << std::put_time(std::localtime(&in_time_t), "%c") << ",";

    // Write data to file
    Logs << message << "\n" << std::flush;
}

void Telemetry::RunTelemetry(Navigation& navigation, Controller& controller, float HardwareSaveDelta) {
        /* Start calculate time change*/
        static auto last_time = std::chrono::high_resolution_clock::now();
        auto time_now = std::chrono::high_resolution_clock::now();
        auto change_time = time_now - last_time;
        /* End calculate time change*/

        
        if(std::chrono::duration_cast<std::chrono::milliseconds>(change_time).count() / 1000.0 >= HardwareSaveDelta){
            HardwareSaveFrame(navigation, controller);
            last_time = std::chrono::high_resolution_clock::now();
        }
}


Telemetry::Telemetry()
{
    Logs.open ("../logs/logs.txt");
    HardwareSaved.open ("../logs/data.txt");;

    //TODO Write headers to data file where needed
}

Telemetry::~Telemetry()
{
    Logs.close();
    HardwareSaved.close();
}