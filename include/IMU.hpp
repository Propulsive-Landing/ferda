#pragma once

#include <tuple>


class IMU
{
    public:
        IMU();
        std::tuple<double, double, double> GetBodyAngularRate(); // Returns angular rate, p, q, and r in order
        std::tuple<double, double, double> GetBodyAcceleration(); // Returns linear acceleration, x, y, z
        void SetGyroBiasX(double x);
        void SetGyroBiasY(double y);
        void SetGyroBiasZ(double z);
    private:
        double gyroBiasX = 0;
        double gyroBiasY = 0;
        double gyroBiasZ = 0;

};
