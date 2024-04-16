#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <stdio.h>
#include <sys/poll.h> 
#include <fstream>

#include "Mode.hpp"
#include "RF.hpp"
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


    RF::GetInstance().SendString(message);

    std::cout << message << "\n";

    Logs << std::put_time(std::localtime(&in_time_t), "%c") << ",";

    // Write data to file
    Logs << message << "\n" << std::flush;
}

void Telemetry::RfSendFrame(Navigation& navigation, Controller& controller)
{
    // write data to rf file
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    RF::rfFrame frame;
    frame.mode = 0;
    frame.euler[0] = navigation.GetNavigation()(0, 0);
    frame.euler[1] = navigation.GetNavigation()(1, 0);
    frame.euler[2] = navigation.GetNavigation()(2, 0);
    frame.velocity[0] = navigation.GetNavigation()(3, 0);
    frame.velocity[1] = navigation.GetNavigation()(4, 0);
    frame.velocity[2] = navigation.GetNavigation()(5, 0);
    frame.dt = 0.0;
    
    RF::GetInstance().SendFrame(frame);

}


void Telemetry::RunTelemetry(Navigation& navigation, Controller& controller, float HardwareSaveDelta, float RFSaveDelta) {
        /* Start calculate time change*/
        static auto last_hardware_time = std::chrono::high_resolution_clock::now();
        auto hardware_change_time = std::chrono::high_resolution_clock::now() - last_hardware_time;

        static auto last_rf_time = std::chrono::high_resolution_clock::now();
        auto rf_change_time = std::chrono::high_resolution_clock::now() - last_rf_time;
        /* End calculate time change*/


        if(std::chrono::duration_cast<std::chrono::milliseconds>(hardware_change_time).count() / 1000.0 >= HardwareSaveDelta){
            HardwareSaveFrame(navigation, controller);
            last_hardware_time = std::chrono::high_resolution_clock::now();
        }
        
        if(std::chrono::duration_cast<std::chrono::milliseconds>(rf_change_time).count() / 1000.0 >= RFSaveDelta){
            RfSendFrame(navigation, controller);
            last_rf_time = std::chrono::high_resolution_clock::now();
        }
}


Telemetry::Telemetry()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto str = oss.str();

    Logs.open ("../logs/logs"+str+".txt");
    HardwareSaved.open ("../logs/data"+str+".txt");;

    //TODO Write headers to data file where needed
}

Telemetry::~Telemetry()
{
    Logs.close();
    HardwareSaved.close();
}