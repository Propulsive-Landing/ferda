#include <tuple>
#include <unistd.h>

#include "IMU.hpp"
#include "Telemetry.hpp"

#include <fstream>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include "MissionConstants.hpp"

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
        // TODO: MAYBE ADD CHECK TO SEE IF USER WANTS TO CONTINUE
        IMUFound = false;
        return;
    }
    else
    {
        IMUFound = true;
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

    return std::make_tuple(nAccelX / 100.0f, nAccelY / 100.0f, nAccelZ / 100.0f);
}

std::tuple<double, double, double> IMU::GetBodyAngularRate()
{

    double nAnglVelX = (double)read16LE(fd, MissionConstants::REG_GYRO_X);
    double nAnglVelY = (double)read16LE(fd, MissionConstants::REG_GYRO_Y);
    double nAnglVelZ = (double)read16LE(fd, MissionConstants::REG_GYRO_Z);

    return std::make_tuple(nAnglVelX / 900.0f, nAnglVelY / 900.0f, nAnglVelZ / 900.0f);
}
