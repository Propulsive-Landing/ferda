#include "Barometer.hpp"
#include "IMU.hpp"
#include "TVC.hpp"

#include "Navigation.hpp"
#include "Controller.hpp"

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

        gpioSetMode(18, PI_OUTPUT);
        gpioSetMode(13, PI_OUTPUT);
    #endif

    IMU imu;
    Barometer barometer;
    TVC tvc;

    Navigation navigation(imu, barometer, tvc);
    Controller controller(tvc);

    Telemetry::GetInstance().Log("Starting program...");

    // TODO we need to set controller iteration gains or there is a segmentation fault.

    //Mode mode(Mode::Calibration);



    // Create a tuple using the tuple constructor
    std::tuple<double,double,double> angularRate = imu.GetBodyAngularRate();
    std::cout << std::get<0>(angularRate) << std::endl; // Output: 1
    std::cout << std::get<1>(angularRate) << std::endl; // Output: 2.5
    std::cout << std::get<2>(angularRate) << std::endl; // Output: Hello



    //while(mode.Update(navigation, controller)) {}

    //#ifdef NDEBUG
    //    gpioTerminate();
   // #endif

    
   


  

    return 0;
}
