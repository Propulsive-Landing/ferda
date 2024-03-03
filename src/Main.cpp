#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"

#include <chrono>
#include <thread>

#include "Navigation.hpp"
#include "Controller.hpp"
#include "Telemetry.hpp"

extern "C" {
    #include "simulation.h"
}

#include "Mode.hpp"

int main()
{
    IMU imu;
    Barometer barometer;
    TVC tvc;
    Igniter igniter;

    Navigation navigation(imu, barometer, tvc, igniter); 
    Controller controller(tvc, igniter);

    simulation_initialize();
    simulation_step();

    using namespace std::this_thread; // sleep_for, sleep_until
    using namespace std::chrono; // nanoseconds, system_clock, seconds


    Mode mode(Mode::Idle);


    Telemetry::GetInstance().SendString("Starting!");

    while( mode.Update(navigation, controller) ) {}

    return 0;
}
