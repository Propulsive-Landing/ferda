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

    IMU imu;
    GPS gps;
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

    while (mode.Update(navigation, controller, igniter, imu))
    {
    }

    // while(1)
    // {
    //     std::cout << "Accel" << "\n";
    //     std::tuple<double,double,double> test = imu.GetBodyAcceleration(); 
    //     std::cout << std::get<0>(test) << ", " << std::get<1>(test) << "," << std::get<2>(test);
    //     std::cout << "\n";

    //     std::cout << "Gyro" << "\n";
    //     std::tuple<double,double,double> test2 = imu.GetBodyAngularRate(); 
    //     std::cout << std::get<0>(test2) << ", " << std::get<1>(test2) << "," << std::get<2>(test2);
    //     std::cout << "\n";
    // }

    // #ifdef NDEBUG
    //     gpioTerminate();
    // #endif

    return 0;
}
