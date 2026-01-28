#include "IMU.hpp"
#include "Magnetometer.hpp"
#include "GPS.hpp"
#include "Camera.hpp"
#include "TVC.hpp"

#include "Navigation.hpp"
#include "Controller.hpp"
#include "Igniter.hpp"
#include "Telemetry.hpp"

#include "Mode.hpp"
#include "MissionConstants.hpp"

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>

#ifdef NDEBUG
#include <pigpio.h>
#endif

int main()
{
#ifdef NDEBUG
    if (gpioInitialise() < 0)
        throw std::runtime_error("failed to initialize gpio");

    gpioSetMode(5, PI_OUTPUT);
    gpioSetMode(6, PI_OUTPUT);

    gpioSetMode(23, PI_OUTPUT);
    gpioSetMode(24, PI_OUTPUT);

    gpioWrite(5, 1);
    gpioWrite(6, 1);

#endif

    IMU imu;
    GPS gps;
    Magnetometer magnetometer;
    Camera camera;
    TVC tvc;
    Igniter igniter;

    Navigation navigation(imu, magnetometer, gps, camera, tvc);
    Controller controller(tvc);

    Telemetry::GetInstance().Log("Starting program...");

    // TODO we need to set controller iteration gains or there is a segmentation fault.

    Mode mode(Mode::Calibration);

    while (mode.Update(navigation, controller, igniter, imu))
    {
    }

    // #ifdef NDEBUG
    //     gpioTerminate();
    // #endif

    return 0;
}
