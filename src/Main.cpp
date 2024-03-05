#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"

#include <chrono>
#include <thread>
#include <iostream>
#include <tuple>

#include "Navigation.hpp"
#include "Controller.hpp"
#include "Telemetry.hpp"

#include "Mode.hpp"

int main()
{
    IMU imu;
    Barometer barometer;
    TVC tvc;
    Igniter igniter;

    Navigation navigation(imu, barometer, tvc, igniter); 
    Controller controller(tvc, igniter);


    using namespace std::this_thread; // sleep_for, sleep_until
    using namespace std::chrono; // nanoseconds, system_clock, seconds


    Mode mode(Mode::Idle);


    Telemetry::GetInstance().SendString("Starting!");

    std::tuple<double, double, double> rates = imu.GetBodyAngularRate();
    double p = std::get<0>(rates);
    std::cout << std::to_string(p) << std::endl;

    // while( mode.Update(navigation, controller) ) {}

    while(true) {} 

    return 0;
}
