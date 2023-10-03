#include "Barometer.hpp"
#include "IMU.hpp"

int main()
{
    IMU imu;
    Barometer barometer;

    barometer.GetPressure();
}
