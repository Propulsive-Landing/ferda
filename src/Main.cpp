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

    Mode mode(Mode::Idle);


    Telemetry::GetInstance().SendString("HELLO");

    while( mode.Update(navigation, controller) ) {
        Telemetry::GetInstance().CheckAndHandleCommand();
    }

    return 0;
}
