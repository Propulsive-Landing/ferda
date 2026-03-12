#pragma once

#include <tuple>
#include <cstdint>

#ifdef NDEBUG
#include <wiringPi.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#endif

class IMU
{
public:
    IMU();
    std::tuple<double, double, double> GetBodyAngularRate();  // Returns angular rate, p, q, and r in order
    std::tuple<double, double, double> GetBodyAcceleration(); // Returns linear acceleration, x, y, z order
    int16_t read16LE(int fd, int reg);                        // Helper: Read 16-bit little-endian
protected:
    int fd; // File pointer
};
