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
#include "RF.hpp"

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
#include "PCA9685Driver.hpp"
#endif

// TODO: add a global ctr c handler to turn off all digital pins

int main()
{
    // Set floating point precision for print statements
    std::cout << std::setprecision(8) << std::fixed;

#ifdef NDEBUG
    if (wiringPiSetupGpio() < 0)
        throw std::runtime_error("failed to initialize gpio");

    // Setup PCA9685 servo driver
    try
    {
        servo_driver = std::make_unique<PiPCA9685::PCA9685>(MissionConstants::PCA9685_I2C_ADDR, MissionConstants::SERVO_DRIVER_ADDR);
        servo_driver->set_pwm_freq(MissionConstants::SERVO_FREQ);
    }
    catch (...)
    {
        // TODO: Log with telemetry
        Telemetry::GetInstance().Log("Warning: Could not setup PCA9685 for Servos");
    }

    // Setup PCA9685 for 1000hz pwm signals for Linear Actuators and Spark plug pwm signal
    try
    {
        pwm_driver = std::make_unique<PiPCA9685::PCA9685>(MissionConstants::PCA9685_I2C_ADDR, MissionConstants::PWM_DRIVER_ADDR);
        pwm_driver->set_pwm_freq(MissionConstants::PWM_FREQ);
    }
    catch (...)
    {
        Telemetry::GetInstance().Log("Warning: Could not setup PCA9685 for Linear Actuators and Spark plug");
    }

    // Setup Analog to Digital Converters
    ads1115Setup(MissionConstants::ADS1BASE, MissionConstants::ADS1ADDR);
    struct wiringPiNodeStruct *node1 = wiringPiFindNode(MissionConstants::ADS1BASE);
    // Attempt to read from config register to see if ADS1115 is connected since ads1115Setup just opens the I2c bus
    int config_value1 = wiringPiI2CReadReg16(node1->fd, 0x01); // Read config register
    if (config_value1 < 0)
    {
        // Log with telemetry
        std::stringstream ss;
        ss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << MissionConstants::ADS1ADDR;
        Telemetry::GetInstance().Log("Warning: Ads1115 was not found at " + ss.str());
    }
    else
    {
        // Set to read up to 6.144 V and upgrade SPS to 860
        digitalWrite(MissionConstants::ADS1BASE, 0);
        digitalWrite(MissionConstants::ADS1BASE + 1, 6);
    }

    ads1115Setup(MissionConstants::ADS2BASE, MissionConstants::ADS2ADDR);
    struct wiringPiNodeStruct *node2 = wiringPiFindNode(MissionConstants::ADS2BASE);
    // Attempt to read from config register to see if ADS1115 is connected since ads1115Setup just opens the I2c bus
    int config_value2 = wiringPiI2CReadReg16(node2->fd, 0x01); // Read config register
    if (config_value2 < 0)
    {
        // Log with telemetry
        std::stringstream ss;
        ss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << MissionConstants::ADS2ADDR;
        Telemetry::GetInstance().Log("Warning: Ads1115 was not found at " + ss.str());
    }
    else
    {
        // Set to read up to 6.144 V and upgrade SPS to 860
        digitalWrite(MissionConstants::ADS2BASE, 0);
        digitalWrite(MissionConstants::ADS2BASE + 1, 6);
    }

    ads1115Setup(MissionConstants::ADS3BASE, MissionConstants::ADS3ADDR);
    struct wiringPiNodeStruct *node3 = wiringPiFindNode(MissionConstants::ADS3BASE);
    // Attempt to read from config register to see if ADS1115 is connected since ads1115Setup just opens the I2c bus
    int config_value3 = wiringPiI2CReadReg16(node3->fd, 0x01); // Read config register
    if (config_value3 < 0)
    {
        // Log with telemetry
        std::stringstream ss;
        ss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << MissionConstants::ADS3ADDR;
        Telemetry::GetInstance().Log("Warning: Ads1115 was not found at " + ss.str());
    }
    else
    {
        // Set to read up to 6.144 V and upgrade SPS to 860
        digitalWrite(MissionConstants::ADS3BASE, 0);
        digitalWrite(MissionConstants::ADS3BASE + 1, 6);
    }

    // TODO: MAYBE ADD USER QUESTON TO SEE IF THEY WANT TO SET PINS EXLCUDING SERVO DRIVER SINCE WE CAN USE A BOOLEAN FOR THAT

    // // Liquid propulsion GPIO setup (solenoid pins)
    // pinMode(MissionConstants::kASIEthanolPin, OUTPUT);
    // pinMode(MissionConstants::kASIOxygenPin, OUTPUT);
    // pinMode(MissionConstants::kNitrogenBleedPin, OUTPUT);

    // // Spark plug pins
    // pinMode(MissionConstants::kSparkPin, OUTPUT);
    // pinMode(MissionConstants::kRPMPin, OUTPUT);

    // // Initialize solenoids to closed state (HIGH for normally-closed, LOW for normally-open)
    // digitalWrite(MissionConstants::kASIEthanolPin, 1);     // HIGH = CLOSED
    // digitalWrite(MissionConstants::kASIOxygenPin, 1);      // HIGH = CLOSED
    // digitalWrite(MissionConstants::kNitrogenBleedPin, 0);  // LOW = CLOSED (normally-open valve)
    // digitalWrite(MissionConstants::kSparkPin, 1);          // HIGH = OFF

    // servo_driver->set_pwm(MissionConstants::kRPMPin, 0, 0); // 0% duty cycle

    // // Initialize servos to closed position (179 degrees)
    // servo_driver->set_pwm(MissionConstants::kNitrogenServoPin, 0, 500 + (MissionConstants::kValveClosedAngle * 2000 / 180));
    // servo_driver->set_pwm(MissionConstants::kPurgeServoPin, 0, 500 + (MissionConstants::kValveClosedAngle * 2000 / 180));
    // servo_driver->set_pwm(MissionConstants::kMainEthanolServoPin, 0, 500 + (MissionConstants::kValveClosedAngle * 2000 / 180));
    // servo_driver->set_pwm(MissionConstants::kMainNitrousServoPin, 0, 500 + (MissionConstants::kValveClosedAngle * 2000 / 180));

#endif
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

    while (mode.Update(navigation, controller, gps, igniter, imu, magnetometer, valveControl,
                       sparkPlug, pressureTransducer, loadCell, camera))
    {
    }

    // #ifdef NDEBUG
    //     gpioTerminate();
    // #endif

    return 0;
}
