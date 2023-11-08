#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"


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

    Telemetry::GetInstance().SendString(std::to_string(simulation_Y.truestate[4]));


    Mode mode(Mode::Idle);


    Telemetry::GetInstance().SendString("Starting!");

    while( mode.Update(navigation, controller) ) {}

    return 0;
}
