#pragma once

#include <tuple>

#include <wiringPiI.h>
#include <wiringPiI2C.h>

class IMU
{
public:
    IMU();
    std::tuple<double, double, double> GetBodyAngularRate();  // Returns angular rate, p, q, and r in order
    std::tuple<double, double, double> GetBodyAcceleration(); // Returns linear acceleration, x, y, z order
    int16_t read16LE(int fd, int reg);                        // Helper: Read 16-bit little-endian
private:
    int fd; // File pointer
};
