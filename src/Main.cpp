#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"

#include "Navigation.hpp"
#include "Controller.hpp"
#include "Telemetry.hpp"

#include "Mode.hpp"

#include <iostream>

int main()
{
    IMU imu;
    Barometer barometer;
    TVC tvc;
    Igniter igniter;

    std::cout << "Pressure: " << barometer.GetPressure() << " kPa\n";
    std::cout << "Temperature: " << barometer.GetTemperature() << " C\n";
    std::cout << "Acceleration: <" << std::get<0>(imu.GetBodyAcceleration()) << ", " << std::get<1>(imu.GetBodyAcceleration()) << ", " << std::get<2>(imu.GetBodyAcceleration()) << "> m/s^2\n";
    std::cout << "Angular Velocity <" << std::get<0>(imu.GetBodyAngularRate()) << ", " << std::get<1>(imu.GetBodyAngularRate()) << ", " << std::get<2>(imu.GetBodyAngularRate()) << "> radians/s\n";

    Navigation navigation(imu, barometer, tvc, igniter);
    Controller controller(tvc, igniter);

    Mode mode(Mode::Idle);


    Telemetry::GetInstance().SendString("Starting!");

    while( mode.Update(navigation, controller) ) {}

    return 0;
}
