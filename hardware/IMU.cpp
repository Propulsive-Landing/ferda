#include <tuple>

#include "IMU.hpp"

#include <fstream>
#include <stdexcept>

#include "MissionConstants.hpp"

namespace {
    std::string gyroPath = "/home/pi/gyroscope_device";
    std::string accelPath = "/home/pi/accel_device";
}


IMU::IMU()
{
    std::fstream ifstream(accelPath + "/in_accel_x_raw");
    if (!ifstream.is_open())
        throw std::runtime_error("accelerometer is not present");
    ifstream.close();

    ifstream = std::fstream(gyroPath + "/in_anglvel_x_raw");
    if (!ifstream.is_open())
        throw std::runtime_error("gyroscope is not present");
    ifstream.close();
}

std::tuple<double, double, double> IMU::GetBodyAcceleration()
{
    std::fstream ifstream(accelPath + "/in_accel_x_raw");
    if (!ifstream.is_open())
        throw std::runtime_error("accelerometer is not present");
    int nAccelX;
    ifstream >> nAccelX;
    ifstream.close();

    ifstream = std::fstream(accelPath + "/in_accel_y_raw");
    if (!ifstream.is_open())
        throw std::runtime_error("accelerometer is not present");
    int nAccelY;
    ifstream >> nAccelY;
    ifstream.close();

    ifstream = std::fstream(accelPath + "/in_accel_z_raw");
    if (!ifstream.is_open())
        throw std::runtime_error("accelerometer is not present");
    int nAccelZ;
    ifstream >> nAccelZ;
    ifstream.close();

    return std::make_tuple(nAccelX * 0.001794, nAccelY * -0.001794, nAccelZ * -0.001794);
}

std::tuple<double, double, double> IMU::GetBodyAngularRate()
{  
    
    std::fstream ifstream(gyroPath + "/in_anglvel_x_raw");
    if (!ifstream.is_open())
        throw std::runtime_error("accelerometer is not present");
    int nAnglVelX;
    ifstream >> nAnglVelX;
    ifstream.close();

    ifstream = std::fstream(gyroPath + "/in_anglvel_y_raw");
    if (!ifstream.is_open())
        throw std::runtime_error("accelerometer is not present");
    int nAnglVelY;
    ifstream >> nAnglVelY;
    ifstream.close();

    ifstream = std::fstream(gyroPath + "/in_anglvel_z_raw");
    if (!ifstream.is_open())
        throw std::runtime_error("accelerometer is not present");
    int nAnglVelZ;
    ifstream >> nAnglVelZ;
    ifstream.close();

    return std::make_tuple(nAnglVelX * 0.000266, -nAnglVelY * 0.000266, -nAnglVelZ * 0.000266);
    
}
