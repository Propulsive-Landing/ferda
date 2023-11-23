#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"

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
    Telemetry telemetry;

    Mode mode(Mode::Idle);


    Telemetry::GetInstance().SendString("Starting!");

    while( mode.Update(navigation, controller, telemetry) ) {}

    return 0;
}
