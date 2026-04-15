#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <stdio.h>
#include <sys/poll.h>
#include <fstream>
#include <tuple>
#include <cstdint>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "Mode.hpp"
#include "RF.hpp"
#include "Telemetry.hpp"
#include "MissionConstants.hpp"

namespace
{
    void WriteElapsedSecondsPrefix(
        std::ofstream &stream,
        const std::chrono::steady_clock::time_point &startTime)
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsedSeconds = std::chrono::duration<double>(now - startTime).count();
        stream << std::fixed << std::setprecision(3) << elapsedSeconds << ", ";
    }
}

void Telemetry::HardwareSaveFrame(Navigation &navigation, Controller &controller, GPS &gps)
{
    WriteElapsedSecondsPrefix(HardwareSaved, StartTime);
    WriteElapsedSecondsPrefix(SensorSaved, StartTime);

    // Navigation state, U, k matrix current index
    // Write data to file

    for (int i = 0; i < 16; ++i)
    {
        HardwareSaved << std::to_string(navigation.GetNavigation()(i)) << ", ";
    }

    std::tuple<double, double, double> gpsPos = gps.GetGPSPosition();
    std::tuple<double, double> gpsVel = gps.GetGPSVelocity();

    HardwareSaved << std::to_string(std::get<0>(gpsPos)) << ", ";
    HardwareSaved << std::to_string(std::get<1>(gpsPos)) << ", ";
    HardwareSaved << std::to_string(std::get<2>(gpsPos)) << ", ";

    HardwareSaved << std::to_string(std::get<0>(gpsVel)) << ", ";
    HardwareSaved << std::to_string(std::get<1>(gpsVel)) << ", ";

    HardwareSaved << std::to_string(controller.GetCurrentTVCCommand()[0]) << ", ";
    HardwareSaved << std::to_string(controller.GetCurrentTVCCommand()[1]) << ", ";

    std::tuple<double, double, double> linAc = navigation.GetLinearAcceleration();
    std::tuple<double, double, double> angAc = navigation.GetAngularAcceleration();
    std::tuple<double, double, double> mag = navigation.GetMagneticField();

    SensorSaved << std::to_string(std::get<0>(linAc)) << ", ";
    SensorSaved << std::to_string(std::get<1>(linAc)) << ", ";
    SensorSaved << std::to_string(std::get<2>(linAc)) << ", ";
    SensorSaved << std::to_string(std::get<0>(angAc)) << ", ";
    SensorSaved << std::to_string(std::get<1>(angAc)) << ", ";
    SensorSaved << std::to_string(std::get<2>(angAc)) << ",";
    SensorSaved << std::to_string(std::get<0>(mag)) << ", ";
    SensorSaved << std::to_string(std::get<1>(mag)) << ", ";
    SensorSaved << std::to_string(std::get<2>(mag));

    HardwareSaved << "\n"
                  << std::flush;
    SensorSaved << "\n"
                << std::flush;
}

void Telemetry::GPSSaveFrame(GPS &gps)
{
    WriteElapsedSecondsPrefix(GPSSaved, StartTime);

    const std::tuple<double, double, double> gpsPos = gps.GetGPSPosition();
    const std::tuple<double, double> gpsVel = gps.GetGPSVelocity();

    GPSSaved << std::to_string(std::get<0>(gpsPos)) << ", ";
    GPSSaved << std::to_string(std::get<1>(gpsPos)) << ", ";
    GPSSaved << std::to_string(std::get<2>(gpsPos)) << ", ";
    GPSSaved << std::to_string(std::get<0>(gpsVel)) << ", ";
    GPSSaved << std::to_string(std::get<1>(gpsVel)) << "\n"
             << std::flush;
}

void Telemetry::Log(std::string message)
{
    // write time to hardware file
    auto time_now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(time_now);

    json json_msg;
    json_msg["data_type"] = "string";
    json_msg["payload"] = message;

    RF::GetInstance().SendString(json_msg.dump() + "\n");

    std::cout << message << "\n";

    // Write data to file
    Logs << std::put_time(std::localtime(&in_time_t), "%c") << ",";
    Logs << message << "\n"
         << std::flush;
}

void Telemetry::RfSendFrame(Navigation &navigation, Controller &controller, PressureTransducer &pt, LoadCell &lc)
{
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
        pt.ReadPSI(PressureTransducer::NitrogenLine),    // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::EthanolTank),     // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::NitrousLine),     // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::OxygenLine),      // 0-200 PSI
        pt.ReadPSI(PressureTransducer::FuelInlet),       // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::FuelOutlet),      // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::ChamberPressure), // 0-1000 PSI
        lc.ReadLBS()                                     // Load cell in pounds
    };

    RF::GetInstance().SendString(json_msg.dump() + "\n");
}

void Telemetry::RunTelemetry(Navigation &navigation, Controller &controller, GPS &gps, PressureTransducer &pt, LoadCell &lc, float HardwareSaveDelta, float RFSaveDelta)
{

    /* Start calculate time change*/
    static auto last_hardware_time = std::chrono::high_resolution_clock::now();
    auto hardware_change_time = std::chrono::high_resolution_clock::now() - last_hardware_time;

    static auto last_rf_time = std::chrono::high_resolution_clock::now();
    auto rf_change_time = std::chrono::high_resolution_clock::now() - last_rf_time;

    static uint64_t last_gps_update_count = 0;
    /* End calculate time change*/

    const uint64_t gps_update_count = gps.GetUpdateCount();
    if (gps_update_count != last_gps_update_count)
    {
        if (gps.GPSAvailable() && gps.GPSUsed())
        {
            GPSSaveFrame(gps);
        }
        last_gps_update_count = gps_update_count;
    }

    if (std::chrono::duration_cast<std::chrono::milliseconds>(hardware_change_time).count() / 1000.0 >= HardwareSaveDelta)
    {
        HardwareSaveFrame(navigation, controller, gps);
        last_hardware_time = std::chrono::high_resolution_clock::now();
    }

    // Log navigational sensors in RFSendFrame() and Log liquid engine sensors in RfSendLiquidPropulsionData()
    if (std::chrono::duration_cast<std::chrono::milliseconds>(rf_change_time).count() / 1000.0 >= RFSaveDelta)
    {
        RfSendFrame(navigation, controller, pt, lc);
        last_rf_time = std::chrono::high_resolution_clock::now();
    }
}

Telemetry::Telemetry() : StartTime(std::chrono::steady_clock::now())
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto str = oss.str();

    Logs.open("../logs/logs" + str + ".txt");
    HardwareSaved.open("../logs/data" + str + ".txt");
    SensorSaved.open("../logs/sensors" + str + ".txt");
    GPSSaved.open("../logs/gps" + str + ".txt");

    HardwareSaved << "TimeSeconds, x, y, z, vx, vy, vz, q1, q2, q3, q4, ab1, ab2, ab3, wb1, wb2, wb3, E, N, U, ux, uy, K_Matrix_Index \n";
    SensorSaved << "TimeSeconds, accelX, accelY, accelZ, gyroX, gryoY, gyroZ, magx, magy, magz \n";
    GPSSaved << "TimeSeconds, gpsE, gpsN, gpsU, gpsVxE, gpsVyN \n";

    HardwareSaved << "Date, x, y, z, vx, vy, vz, q1, q2, q3, q4, ab1, ab2, ab3, wb1, wb2, wb3, E, N, U, E_Vel, N_Vel, ux, uy \n";
    SensorSaved << "Date, accelX, accelY, accelZ, gyroX, gryoY, gyroZ, magx, magy, magz \n";
}

Telemetry::~Telemetry()
{
    Logs.close();
    HardwareSaved.close();
    SensorSaved.close();
    GPSSaved.close();
}
