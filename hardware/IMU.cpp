#include <tuple>
#include <unistd.h>

#include "IMU.hpp"
#include "Telemetry.hpp"

#include <fstream>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "MissionConstants.hpp"

namespace
{
    bool IsRightAngleAxisMap(const Eigen::Matrix3i &axisMap)
    {
        for (int row = 0; row < 3; ++row)
        {
            int nonZeroCount = 0;
            for (int col = 0; col < 3; ++col)
            {
                const int value = axisMap(row, col);
                if (value != 0)
                {
                    if (value != 1 && value != -1)
                    {
                        return false;
                    }
                    ++nonZeroCount;
                }
            }
            if (nonZeroCount != 1)
            {
                return false;
            }
        }

        for (int col = 0; col < 3; ++col)
        {
            int nonZeroCount = 0;
            for (int row = 0; row < 3; ++row)
            {
                if (axisMap(row, col) != 0)
                {
                    ++nonZeroCount;
                }
            }
            if (nonZeroCount != 1)
            {
                return false;
            }
        }

        const int determinant = axisMap.determinant();
        return std::abs(determinant) == 1;
    }

    Eigen::Vector3d MapImuSensorToBody(const Eigen::Vector3d &sensorVector)
    {
        static const bool kAxisMapValid = IsRightAngleAxisMap(MissionConstants::kSensorImuBodyAxisMap);
        static bool warnedInvalidAxisMap = false;

        if (!kAxisMapValid)
        {
            if (!warnedInvalidAxisMap)
            {
                Telemetry::GetInstance().Log("Warning: kSensorImuBodyAxisMap is invalid. Falling back to identity IMU axis mapping.");
                warnedInvalidAxisMap = true;
            }
            return sensorVector;
        }

        return MissionConstants::kSensorImuBodyAxisMap.cast<double>() * sensorVector;

    }
}

IMU::IMU()
{
    fd = wiringPiI2CSetup(MissionConstants::IMU_I2C_ADDR);

    int chip_id = wiringPiI2CReadReg8(fd, MissionConstants::CHIP_ID_ADDR);
    std::stringstream ss;
    ss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << MissionConstants::IMU_I2C_ADDR;

    // Check to see if BMO055 is connected since wiringPiI2CSetup just opens up I2c bus
    if (chip_id != MissionConstants::CHIP_ID)
    {
        Telemetry::GetInstance().Log("Warning: BMO055 was not detected at address " + ss.str());
        return;
    }

    // Set power mode
    wiringPiI2CWriteReg8(fd, MissionConstants::POWER_MODE, MissionConstants::POWER_NORMAL);
    delay(10);

    // Set operation mode to AMG to get raw accelerometer, gyroscope, and magnometer data
    wiringPiI2CWriteReg8(fd, MissionConstants::OPERATION_MODE, MissionConstants::CONFIG);
    delay(50); // allow sensor to start
    wiringPiI2CWriteReg8(fd, MissionConstants::OPERATION_MODE, MissionConstants::AMG);
    delay(50); // allow sensor to start
    wiringPiI2CWriteReg8(fd, MissionConstants::UNIT_SEL, MissionConstants::RAD);
    delay(50); // allow sensor to start
}
int16_t IMU::read16LE(int fd, int reg)
{
    uint8_t buffer[2];

    // Write register address to set read position
    if (write(fd, &reg, 1) != 1)
    {
        return 0;
    }

    // Read 2 consecutive bytes in a single atomic I2C transaction
    if (read(fd, buffer, 2) != 2)
    {
        return 0;
    }

    return (int16_t)((buffer[1] << 8) | buffer[0]);
}

std::tuple<double, double, double> IMU::GetBodyAcceleration()
{

    double nAccelX = (double)read16LE(fd, MissionConstants::REG_ACC_X);
    double nAccelY = (double)read16LE(fd, MissionConstants::REG_ACC_Y);
    double nAccelZ = (double)read16LE(fd, MissionConstants::REG_ACC_Z);

    const Eigen::Vector3d accelSensor(
        nAccelX / 100.0f,
        nAccelY / 100.0f,
        nAccelZ / 100.0f);
    const Eigen::Vector3d accelBody = MapImuSensorToBody(accelSensor - MissionConstants::kSensorImuAccelBiasSensorMps2);

    return std::make_tuple(accelBody.x(), accelBody.y(), accelBody.z());
}

std::tuple<double, double, double> IMU::GetBodyAngularRate()
{

    double nAnglVelX = (double)read16LE(fd, MissionConstants::REG_GYRO_X);
    double nAnglVelY = (double)read16LE(fd, MissionConstants::REG_GYRO_Y);
    double nAnglVelZ = (double)read16LE(fd, MissionConstants::REG_GYRO_Z);

    const Eigen::Vector3d gyroSensor(nAnglVelX / 900.0f, nAnglVelY / 900.0f, nAnglVelZ / 900.0f);
    const Eigen::Vector3d gyroBody = MapImuSensorToBody(gyroSensor);

    return std::make_tuple(gyroBody.x(), gyroBody.y(), gyroBody.z());
}
