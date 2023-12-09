#include "Barometer.hpp"
#include "IMU.hpp"
#include "TVC.hpp"

#include "Navigation.hpp"
#include "Controller.hpp"

#include "Telemetry.hpp"

#include "Mode.hpp"

#include <iostream>
#include <stdexcept>

#ifdef NDEBUG
    #include <pigpio.h>
#endif

int main()
{
    #ifdef NDEBUG
        if (gpioInitialise() < 0)
            throw std::runtime_error("failed to initialize gpio");

        gpioSetMode(18, PI_OUTPUT);
        gpioSetMode(5, PI_OUTPUT);
    #endif

    IMU imu;
    Barometer barometer;
    TVC tvc;

    Navigation navigation(imu, barometer, tvc);
    Controller controller(tvc);

    Telemetry::GetInstance().Log("Starting program...");


    // TODO we need to set controller iteration gains or there is a segmentation fault.

    Mode mode(Mode::TestTVC);

    while(mode.Update(navigation, controller)) {}

    #ifdef NDEBUG
        gpioTerminate();
    #endif

    return 0;
}
