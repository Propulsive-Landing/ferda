#include <tuple>
#include <unistd.h>

#include "IMU.hpp"

#include <fstream>
#include <stdexcept>
#include "MissionConstants.hpp"

IMU::IMU()
{
    fd = wiringPiI2CSetup(MissionConstants::IMU_i2c_addr);
    if (fd == -1)
    {
        std::cerr << "BNO055 not found!" << std::endl;
        exit(-1);
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
    if (write(fd, &reg, 1) != 1) {
        return 0;
    }
    
    // Read 2 consecutive bytes in a single atomic I2C transaction
    if (read(fd, buffer, 2) != 2) {
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

    return std::make_tuple(nAnglVelX / 900.0f, nAnglVelY / 900.0f, nAnglVelZ / 900.0f); // deg/s
}
