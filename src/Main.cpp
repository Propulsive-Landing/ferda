#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
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

    gpioSetMode(4, PI_OUTPUT);
    gpioSetMode(17, PI_OUTPUT);

    gpioWrite(4, 0);
    gpioWrite(17, 0);

    gpioSetMode(18, PI_OUTPUT);
    gpioSetMode(5, PI_OUTPUT);
#endif

    IMU imu;
    Barometer barometer;
    TVC tvc;
    Igniter igniter;

    Navigation navigation(imu, barometer, tvc, igniter);
    Controller controller(tvc, igniter);

    Mode mode(Mode::TestTVC);

    while( mode.Update(navigation, controller) ) {}

#ifdef NDEBUG
    gpioTerminate();
#endif
    return 0;
}
