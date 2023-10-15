#include <iostream>

#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"

#include "Navigation.hpp"
#include "Controller.hpp"

#include "Mode.hpp"

int main()
{
    IMU imu;
    Barometer barometer;
    TVC tvc;
    Igniter igniter;

    Navigation navigation(imu, barometer, tvc, igniter); 
    Controller controller(tvc, igniter);

    Mode mode(Mode::Idle);

    std::cout << mode.Update(navigation, controller) << "\n";
    std::cout << mode.Update(navigation, controller) << "\n";
    std::cout << mode.Update(navigation, controller) << "\n";
    std::cout << mode.Update(navigation, controller) << "\n";
}
