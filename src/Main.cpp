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
#include <ads1115.h>
#include "ServoDriver.hpp"
#endif

int main()
{
#ifdef NDEBUG
    if (wiringPiSetupGpio() < 0)
        throw std::runtime_error("failed to initialize gpio");

    // Setup Analog to Digital Converters
    if (ads1115Setup(ADS1BASE, ADS1ADDR) < 0)
    {
        // TODO: Maybe try to find. a way to use Telemetry to log and ask user if they want to abort
        std::cerr << "Warning: Ads1115 was not found at " << ADS1ADDR;
    }
    else
    {
        // Set to read up to 6.144 V
        digitalWrite(MissionConstants::ADS1BASE, 0)
    }

    if (ads1115Setup(ADS2BASE, ADS2ADDR) < 0)
    {
        // TODO: Maybe try to find. a way to use Telemetry to log and ask user if they want to abort
        std::cerr << "Warning: Ads1115 was not found at " << ADS1ADDR;
    }
    else
    {
        digitalWrite(MissionConstants::ADS2BASE, 0)
    }

    if (ads1115Setup(ADS3BASE, ADS3ADDR) < 0)
    {
        // TODO: Maybe try to find. a way to use Telemetry to log and ask user if they want to abort
        std::cerr << "Warning: Ads1115 was not found at " << ADS1ADDR;
    }
    else
    {
        digitalWrite(MissionConstants::ADS2BASE, 0)
    }

    // Liquid propulsion GPIO setup (solenoid pins)
    pinMode(MissionConstants::kASIEthanolPin, PI_OUTPUT);
    pinMode(MissionConstants::kASIOxygenPin, PI_OUTPUT);
    pinMode(MissionConstants::kNitrogenBleedPin, PI_OUTPUT);

    // Spark plug pins
    pinMode(MissionConstants::kSparkPin, PI_OUTPUT);
    pinMode(MissionConstants::kRPMPin, PI_OUTPUT);

    // Initialize solenoids to closed state (HIGH for normally-closed, LOW for normally-open)
    digitalWrite(MissionConstants::kASIEthanolPin, 1);     // HIGH = CLOSED
    digitalWrite(MissionConstants::kASIOxygenPin, 1);      // HIGH = CLOSED
    digitalWrite(MissionConstants::kNitrogenBleedPin, 0);  // LOW = CLOSED (normally-open valve)
    digitalWrite(MissionConstants::kSparkPin, 1);          // HIGH = OFF
    servo_driver.set_pwm(MissionConstants::kRPMPin, 0, 0); // 0% duty cycle

    // Initialize servos to closed position (179 degrees)
    servo_driver.set_pwm(MissionConstants::kNitrogenServoPin, 0, 500 + (MissionConstants::kValveClosedAngle * 2000 / 180));
    servo_driver.set_pwm(MissionConstants::kPurgeServoPin, 0, 500 + (MissionConstants::kValveClosedAngle * 2000 / 180));
    servo_driver.set_pwm(MissionConstants::kMainEthanolServoPin, 0, 500 + (MissionConstants::kValveClosedAngle * 2000 / 180));
    servo_driver.set_pwm(MissionConstants::kMainNitrousServoPin, 0, 500 + (MissionConstants::kValveClosedAngle * 2000 / 180));

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
