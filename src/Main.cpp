#include <iostream>

#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"

#include "State.hpp"

int main()
{
    IMU imu;
    Barometer barometer;
    TVC tvc;
    Igniter igniter;
    State state(State::Idle);

    std::cout << state.Update(imu, barometer, tvc, igniter) << "\n";
    std::cout << state.Update(imu, barometer, tvc, igniter) << "\n";
    std::cout << state.Update(imu, barometer, tvc, igniter) << "\n";
    std::cout << state.Update(imu, barometer, tvc, igniter) << "\n";
}
