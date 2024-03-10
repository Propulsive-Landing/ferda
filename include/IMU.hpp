#pragma once

#include <tuple>
#include <Eigen/Dense>


class IMU
{
    public:
        IMU();
         // For testing
       
        std::tuple<double, double, double> GetBodyAngularRate(); // Returns angular rate, p, q, and r in order
        std::tuple<double, double, double> GetBodyAcceleration(); // Returns linear acceleration, x, y, z
};
