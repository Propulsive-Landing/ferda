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
#include <unistd.h>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>

#ifdef NDEBUG
#include <wiringPi.h>
#endif

int main()
{
#ifdef NDEBUG
    if (wiringPiSetupGpio() < 0)
        throw std::runtime_error("failed to initialize gpio");
    // ADD NEW PIN SETUPS BECAUSE WE DON'T NEED THE ONES BELOW
    // pinMode(5, OUTPUT);
    // pinMode(6, OUTPUT);

    // pinMode(23, OUTPUT);
    // pinMode(24, OUTPUT);

    // gpioWrite(5, 1);
    // gpioWrite(6, 1);

#endif
    std::cout << std::setprecision(8) << std::fixed;
    IMU imu;
    GPS gps;
    sleep(2); // Allow gps to wake up
    std::string settings = std::string("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
    gps.write_settings(settings);
    gps.wait_for_confirmation(std::string("314"));
    // settings = std::string("$PMTK220,100*2F\r\n");
    // gps.write_settings(settings);
    // gps.wait_for_confirmation(std::string("220"));
    gps.reset_acculumated_messages();

    Magnetometer magnetometer;
    Camera camera;
    TVC tvc;
    Igniter igniter;
    Engine engine;

    Navigation navigation(imu, magnetometer, gps, camera, tvc);
    Controller controller(tvc, engine);

    Telemetry::GetInstance().Log("Starting program...");

    // TODO we need to set controller iteration gains or there is a segmentation fault.

    Mode mode(Mode::Calibration);

    while (mode.Update(navigation, controller, gps, igniter, imu))
    {
    }

    // #ifdef NDEBUG
    //     gpioTerminate();
    // #endif

    return 0;
}
