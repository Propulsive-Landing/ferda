#include "Barometer.hpp"
#include "IMU.hpp"
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

    // Solid propulsion GPIO setup
    gpioSetMode(5, PI_OUTPUT);
    gpioSetMode(6, PI_OUTPUT);
    gpioSetMode(23, PI_OUTPUT);
    gpioSetMode(24, PI_OUTPUT);
    gpioWrite(5, 1);
    gpioWrite(6, 1);

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
    gpioWrite(MissionConstants::kASIEthanolPin, 1); // HIGH = CLOSED
    gpioWrite(MissionConstants::kASIOxygenPin, 1);    // HIGH = CLOSED
    gpioWrite(MissionConstants::kNitrogenBleedPin, 0); // LOW = CLOSED (normally-open valve)
    gpioWrite(MissionConstants::kSparkPin, 1);       // HIGH = OFF
    gpioPWM(MissionConstants::kRPMPin, 0);          // 0% duty cycle
    
    // Initialize servos to closed position (179 degrees)
    gpioServo(MissionConstants::kNitrogenServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
    gpioServo(MissionConstants::kPurgeServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
    gpioServo(MissionConstants::kMainEthanolServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
    gpioServo(MissionConstants::kMainNitrousServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));

#endif

    IMU imu;
    Barometer barometer;
    TVC tvc;
    Igniter igniter;

    Navigation navigation(imu, barometer, tvc);
    Controller controller(tvc);

    Telemetry::GetInstance().Log("Starting program...");

    // TODO we need to set controller iteration gains or there is a segmentation fault.

    // Initialize liquid propulsion hardware (optional - only used in liquid states)
    // These can be nullptr for solid propulsion missions
    ValveControl valveControl;
    SparkPlug sparkPlug;
    PressureTransducer pressureTransducer;
    LoadCell loadCell;

    Mode mode(Mode::Calibration);
    
    // Set liquid propulsion hardware in Mode (can be used when entering liquid states)
    mode.SetLiquidPropulsionHardware(&valveControl, &sparkPlug, &pressureTransducer, &loadCell);

    while (mode.Update(navigation, controller, igniter, imu))
    {
    }

    // #ifdef NDEBUG
    //     gpioTerminate();
    // #endif

    return 0;
}
