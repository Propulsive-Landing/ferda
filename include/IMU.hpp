#include <tuple>


class IMU
{
    public:
        IMU() = default;
        tuple<double, double, double> GetBodyAngularRate(); // Returns angular rate, p, q, and r in order
        tuple<double, double, double> GetBodyAcceleration(); // Returns linear acceleration, x, y, z
};