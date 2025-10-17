#include <tuple>

#include "IMU.hpp"

#include <fstream>
#include <stdexcept>

#include "MissionConstants.hpp"

namespace
{
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

void IMU::SetGyroBiasX(double x)
{
    gyroBiasX = x;
}
void IMU::SetGyroBiasY(double y)
{
    gyroBiasY = y;
}
void IMU::SetGyroBiasZ(double z)
{
    gyroBiasZ = z;
}

void IMU::SetAccelBiasX(double x)
{
    accelBiasX = x;
}
void IMU::SetAccelBiasY(double y)
{
    accelBiasY = y;
}
void IMU::SetAccelBiasZ(double z)
{
    accelBiasZ = z;
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

    return std::make_tuple(nAccelX * 0.001794 + accelBiasX, nAccelY * -0.001794 + accelBiasY, nAccelZ * -0.001794 + accelBiasZ);
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

    return std::make_tuple(nAnglVelX * 0.000266 + gyroBiasX, -nAnglVelY * 0.000266 + gyroBiasY, -nAnglVelZ * 0.000266 + gyroBiasZ);
}
