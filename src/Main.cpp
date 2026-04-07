#include "IMU.hpp"
#include "Magnetometer.hpp"
#include "GPS.hpp"
#include "Lidar.hpp"
#include "Camera.hpp"
#include "TVC.hpp"

#include "Navigation.hpp"
#include "Controller.hpp"
#include "Igniter.hpp"
#include "Telemetry.hpp"

#include "Mode.hpp"
#include "MissionConstants.hpp"
#include "ValveControl.hpp"
#include "SparkPlug.hpp"
#include "PressureTransducer.hpp"
#include "LoadCell.hpp"

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

    // Liquid propulsion GPIO setup (servo pins)
    gpioSetMode(MissionConstants::kNitrogenServoPin, PI_OUTPUT);
    gpioSetMode(MissionConstants::kPurgeServoPin, PI_OUTPUT);
    gpioSetMode(MissionConstants::kMainEthanolServoPin, PI_OUTPUT);
    gpioSetMode(MissionConstants::kMainNitrousServoPin, PI_OUTPUT);

    // Liquid propulsion GPIO setup (solenoid pins)
    gpioSetMode(MissionConstants::kASIEthanolPin, PI_OUTPUT);
    gpioSetMode(MissionConstants::kASIOxygenPin, PI_OUTPUT);
    gpioSetMode(MissionConstants::kNitrogenBleedPin, PI_OUTPUT);

    // Spark plug pins
    gpioSetMode(MissionConstants::kSparkPin, PI_OUTPUT);
    gpioSetMode(MissionConstants::kRPMPin, PI_OUTPUT);

    // Initialize solenoids to closed state (HIGH for normally-closed, LOW for normally-open)
    gpioWrite(MissionConstants::kASIEthanolPin, 1);    // HIGH = CLOSED
    gpioWrite(MissionConstants::kASIOxygenPin, 1);     // HIGH = CLOSED
    gpioWrite(MissionConstants::kNitrogenBleedPin, 0); // LOW = CLOSED (normally-open valve)
    gpioWrite(MissionConstants::kSparkPin, 1);         // HIGH = OFF
    gpioPWM(MissionConstants::kRPMPin, 0);             // 0% duty cycle

    // Initialize servos to closed position (179 degrees)
    gpioServo(MissionConstants::kNitrogenServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
    gpioServo(MissionConstants::kPurgeServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
    gpioServo(MissionConstants::kMainEthanolServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
    gpioServo(MissionConstants::kMainNitrousServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));

#endif
    std::cout << std::setprecision(8) << std::fixed;
    IMU imu;
    GPS gps;
    Lidar lidar;
    Magnetometer magnetometer;
    Camera camera;
    TVC tvc;
    Igniter igniter;
    Engine engine;

    Navigation navigation(imu, magnetometer, gps, lidar, camera, tvc);
    Controller controller(tvc, engine);

    // Initialize liquid propulsion hardware
    ValveControl valveControl;
    SparkPlug sparkPlug;
    PressureTransducer pressureTransducer;
    LoadCell loadCell;

    Mode mode(Mode::Calibration);

    Telemetry::GetInstance().Log("Starting program...");

    while (mode.Update(navigation, controller, gps, igniter, imu, valveControl, sparkPlug, pressureTransducer, loadCell))
    {
    }

    // #ifdef NDEBUG
    //     gpioTerminate();
    // #endif

    return 0;
}
