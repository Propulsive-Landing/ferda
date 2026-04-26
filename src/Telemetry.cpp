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
#include <cmath>

#include <Eigen/Geometry>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "Mode.hpp"
#include "RF.hpp"
#include "Telemetry.hpp"
#include "MissionConstants.hpp"

namespace
{
    double RoundForRf(double value, int decimalPlaces)
    {
        const double scale = std::pow(10.0, decimalPlaces);
        return std::round(value * scale) / scale;
    }

    json BuildRoundedPayload(std::initializer_list<double> values, int decimalPlaces)
    {
        json payload = json::array();
        for (const double value : values)
        {
            payload.push_back(RoundForRf(value, decimalPlaces));
        }
        return payload;
    }

    Eigen::Vector3d QuaternionToEulerXyzRad(const Eigen::Quaterniond &q)
    {
        const Eigen::Quaterniond qNormalized = q.normalized();
        // Returns roll, pitch, yaw in radians for XYZ sequence.
        return qNormalized.toRotationMatrix().eulerAngles(0, 1, 2);
    }

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

void Telemetry::LogActuatorFrame(double commanded_angle_x_rad,
                                 double commanded_angle_y_rad,
                                 double commanded_length_x_in,
                                 double commanded_length_y_in,
                                 double observed_length_x_in,
                                 double observed_length_y_in,
                                 int commanded_speed_x,
                                 int commanded_speed_y)
{
    WriteElapsedSecondsPrefix(ActuatorSaved, StartTime);

    ActuatorSaved << commanded_angle_x_rad << ", ";
    ActuatorSaved << commanded_angle_y_rad << ", ";
    ActuatorSaved << commanded_length_x_in << ", ";
    ActuatorSaved << commanded_length_y_in << ", ";
    ActuatorSaved << observed_length_x_in << ", ";
    ActuatorSaved << observed_length_y_in << ", ";
    ActuatorSaved << commanded_speed_x << ", ";
    ActuatorSaved << commanded_speed_y << "\n"
                  << std::flush;
}

void Telemetry::Log(std::string message)
{
    // write time to hardware file
    auto now = std::chrono::system_clock::now();

    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();

    std::time_t tt = std::chrono::system_clock::to_time_t(seconds);

    std::tm tm;
    localtime_r(&tt, &tm); // thread-safe on Linux

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms;

    json json_msg;
    json_msg["data_type"] = "string";
    json_msg["payload"] = message;

    RF::GetInstance().SendString(json_msg.dump() + "\n");

    std::cout << message << "\n";

    // Write data to file
    Logs << oss.str() << ", ";
    Logs << message << "\n"
         << std::flush;
}

void Telemetry::RfSendGNCFrame(Navigation &navigation, Controller &controller)
{
    // TODO: MAKE SURE THIS FOLLOWS WHAT GROUND CONTROL EXPECTS
    const Eigen::Matrix<double, 16, 1> navState = navigation.GetNavigation();
    const Eigen::Vector3d angularVelocity = navigation.GetAngularVelocity();
    const Eigen::Quaterniond q(
        navState(6), // w
        navState(7), // x
        navState(8), // y
        navState(9)  // z
    );
    const Eigen::Vector3d eulerXyz = QuaternionToEulerXyzRad(q);
    const Eigen::Vector2d actuatorSetpointError = controller.tvc.GetActuatorSetpointErrorInches();
    const Eigen::Vector3d attitudeSetpointError = controller.GetCurrentAttitudeSetpointError();
    const double guidanceAltitudeError = controller.GetCurrentGuidanceAltitudeError();
    const Eigen::Vector2d guidanceTranslationError = controller.GetCurrentGuidanceTranslationError();
    const int decimalPlaces = MissionConstants::kRfGncPayloadDecimalPlaces;

    json json_msg;
    json_msg["data_type"] = "telem";
    json_msg["type"] = "GNC";
    json_msg["payload"] = BuildRoundedPayload({
        // Position
        navState(0),
        navState(1),
        navState(2),
        // Velocity
        navState(3),
        navState(4),
        navState(5),
        // Eulers
        eulerXyz(0),
        eulerXyz(1),
        eulerXyz(2),
        // Omegas
        angularVelocity(0),
        angularVelocity(1),
        angularVelocity(2),
        // Accel Bias
        navState(10),
        navState(11),
        navState(12),
        // Omega Bias
        navState(13),
        navState(14),
        navState(15),
        // Tvc commands
        controller.GetCurrentTVCCommand()(0),
        controller.GetCurrentTVCCommand()(1),
        // Rcs command
        controller.GetCurrentRcsCommand(),
        // Thrust command,
        controller.GetCurrentThrustCommand(),
        // Actuator setpoint errors
        actuatorSetpointError(0),
        actuatorSetpointError(1),
        // Attitude Setpoint errors
        attitudeSetpointError(0),
        attitudeSetpointError(1),
        attitudeSetpointError(2),
        // Guidance altitude error
        guidanceAltitudeError,
        // Guidiance translation errors
        guidanceTranslationError(0),
        guidanceTranslationError(1)

    }, decimalPlaces);

    std::cout << "GNC Telemetry: " << json_msg["payload"].size() << std::endl;
    RF::GetInstance().SendString(json_msg.dump() + "\n");
}

void Telemetry::RfSendLiquidFrame(PressureTransducer &pt, LoadCell &lc)
{
    const int decimalPlaces = MissionConstants::kRfLiquidPayloadDecimalPlaces;

    json json_msg;
    json_msg["data_type"] = "telem";
    json_msg["type"] = "Liquid";
    json_msg["payload"] = BuildRoundedPayload({
        pt.ReadPSI(PressureTransducer::NitrogenLine),    // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::EthanolTank),     // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::NitrousLine),     // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::OxygenLine),      // 0-200 PSI
        pt.ReadPSI(PressureTransducer::FuelInlet),       // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::FuelOutlet),      // 0-1000 PSI
        pt.ReadPSI(PressureTransducer::ChamberPressure), // 0-1000 PSI
        lc.ReadLBS()                                     // Load cell in pounds
    }, decimalPlaces);

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
        if (gps.GPSAvailable())
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
        RfSendGNCFrame(navigation, controller);
        RfSendLiquidFrame(pt, lc);
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
    ActuatorSaved.open("../logs/actuators" + str + ".txt");

    HardwareSaved << "TimeSeconds, x, y, z, vx, vy, vz, q1, q2, q3, q4, ab1, ab2, ab3, wb1, wb2, wb3, E, N, U, E_Vel, N_Vel, ux, uy \n";
    SensorSaved << "TimeSeconds, accelX, accelY, accelZ, gyroX, gryoY, gyroZ, magx, magy, magz \n";
    GPSSaved << "TimeSeconds, gpsE, gpsN, gpsU, gpsVxE, gpsVyN \n";
    ActuatorSaved << "TimeSeconds, commandedAngleXRad, commandedAngleYRad, commandedLengthXIn, commandedLengthYIn, observedLengthXIn, observedLengthYIn, commandedSpeedX, commandedSpeedY \n";
}

Telemetry::~Telemetry()
{
    Logs.close();
    HardwareSaved.close();
    SensorSaved.close();
    GPSSaved.close();
    ActuatorSaved.close();
}
