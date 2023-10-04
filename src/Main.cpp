#include <iostream>

#include "Barometer.hpp"
#include "IMU.hpp"
#include "State.hpp"

int main()
{
    IMU imu;
    Barometer barometer;
    State state(State::Idle);

    std::cout << state.Update() << "\n";
    std::cout << state.Update() << "\n";
    std::cout << state.Update() << "\n";
    std::cout << state.Update() << "\n";
}
