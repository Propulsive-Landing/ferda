#include <iostream>

#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"

#include "Navigation.hpp"
#include "Controller.hpp"

#include "State.hpp"

int main()
{
    IMU imu;
    Barometer barometer;
    TVC tvc;
    Igniter igniter;

    Navigation navigation(imu, barometer, tvc, igniter); 
    Controller controller(tvc, igniter);

    State state(State::Idle);

    std::cout << state.Update(navigation, controller) << "\n";
    std::cout << state.Update(navigation, controller) << "\n";
    std::cout << state.Update(navigation, controller) << "\n";
    std::cout << state.Update(navigation, controller) << "\n";
}
