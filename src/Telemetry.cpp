#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <stdio.h>
#include <sys/poll.h> 
#include <fstream>
#include <tuple>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "Mode.hpp"
#include "RF.hpp"
#include "Telemetry.hpp"
#include "MissionConstants.hpp"

void Telemetry::HardwareSaveFrame(Navigation& navigation, Controller& controller)
{
    // write time to hardware file
    auto time_now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(time_now);

    HardwareSaved << std::put_time(std::localtime(&in_time_t), "%c") << ", " ;
    SensorSaved << std::put_time(std::localtime(&in_time_t), "%c") << ", " ;

    // Navigation state, U, k matrix current index
    // Write data to file
    

    for(int i = 0; i < 12; ++i){
        HardwareSaved << std::to_string(navigation.GetNavigation()(i))<< ", ";
    }
    HardwareSaved << std::to_string(controller.GetCurrentIterationIndex());

    std::tuple<double, double, double> linAc = navigation.GetLinearAcceleration();
    std::tuple<double, double, double> angAc = navigation.GetAngularAcceleration();
    SensorSaved << std::to_string(std::get<0>(linAc))<< ", ";
    SensorSaved << std::to_string(std::get<1>(linAc))<< ", ";
    SensorSaved << std::to_string(std::get<2>(linAc))<< ", ";
    SensorSaved << std::to_string(std::get<0>(angAc))<< ", ";
    SensorSaved << std::to_string(std::get<1>(angAc))<< ", ";
    SensorSaved << std::to_string(std::get<2>(angAc));

    HardwareSaved<<"\n" << std::flush;
    SensorSaved<<"\n" << std::flush;

}


void Telemetry::Log(std::string message) {
    // write time to hardware file
    auto time_now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(time_now);

    json json_msg;
    json_msg["data_type"] = "string";
    json_msg["payload"] = message;

    RF::GetInstance().SendString(json_msg.dump());

    std::cout << message << "\n";

    // Write data to file
    Logs << std::put_time(std::localtime(&in_time_t), "%c") << ",";
    Logs << message << "\n" << std::flush;
} 

void Telemetry::RfSendFrame(Navigation& navigation, Controller& controller)
{
    // write data to rf file
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    json json_msg;
    json_msg["data_type"] = "telem";
    json_msg["payload"] = {
        // Euler.
        navigation.GetNavigation()(0, 0),
        navigation.GetNavigation()(1, 0),
        navigation.GetNavigation()(2, 0),

        // Input.
        controller.input(0),
        controller.input(1),

        // Velocity.
        navigation.GetNavigation()(3, 0),
        navigation.GetNavigation()(4, 0),
        navigation.GetNavigation()(5, 0),

        // dt.
        0.0,
    };
    
    // frame.euler[0] = 1.0;// navigation.GetNavigation()(0, 0);
    // frame.euler[1] = 2.0; // navigation.GetNavigation()(1, 0);
    // frame.euler[2] = 3.0; // navigation.GetNavigation()(2, 0);
    // frame.input[0] = 5.0; // controller.input(0);
    // frame.input[1] = 6.0; // controller.input(1);
    // frame.velocity[0] = 7.0; // navigation.GetNavigation()(3, 0);
    // frame.velocity[1] = 8.0; // navigation.GetNavigation()(4, 0);
    // frame.velocity[2] = 9.0; // navigation.GetNavigation()(5, 0);
    // frame.dt = 0.0;
    
    RF::GetInstance().SendString(json_msg.dump());
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
    HardwareSaved.open ("../logs/data"+str+".txt");
    SensorSaved.open ("../logs/sensors"+str+".txt");


    HardwareSaved << "Date, x, y, z, vx, vy, vz, phi, theta, psi, p, q, r, K_Matrix_Index \n";
    SensorSaved << "Date, accelX, accelY, accelZ, gyroX, gryoY, gyroZ \n";



    //TODO Write headers to data file where needed
}

Telemetry::~Telemetry()
{
    Logs.close();
    HardwareSaved.close();
}
