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


    simulation_U.tvcxangle = 0.0;                    /* '<Root>/tvc x angle' */
    simulation_U.tvcyangle = 0.0;                     /* '<Root>/tvc y angle' */
    simulation_U.ignite_s1 = 1.0;                     /* '<Root>/ignite_s1' */
    simulation_U.ignite_s2 = 0.0;                     /* '<Root>/ignite_s2' */

    while(true){
        simulation_step();
        Telemetry::GetInstance().SendString(std::to_string(simulation_Y.acceleration[2]));
        Telemetry::GetInstance().SendString(std::to_string(simulation_DW.is_c3_simulation));
        sleep_for(nanoseconds(1000000));
    }


    Mode mode(Mode::Idle);


    Telemetry::GetInstance().SendString("Starting!");

    while( mode.Update(navigation, controller) ) {}

    return 0;
}
