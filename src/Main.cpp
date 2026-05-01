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
#include <csignal>

#ifdef NDEBUG
#include <wiringPi.h>
#include <ads1115.h>
#include "PCA9685Driver.hpp"
#endif

static volatile sig_atomic_t stopRequested = 0;

void signalHanlder(int sig)
{
    stopRequested = 1;
}

int main()
{
    // Set signal action behvaior for when user does ctr+c
    struct sigaction sa{};
    sa.sa_handler = signalHanlder;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    sigaction(SIGINT, &sa, nullptr);

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

    /*
    TODO: ADDRESS HOW TO HANDLE IF USER CONTINUES WITHOUT WARNING MESSAGES:
           PARTICULARLY THE PCA9685 VARIABLES SINCE THOSE WILL BE NULL POINTERS. I ADDED A BOOLEAN CHECK
            IN driveActuator() IN LinActMotorPositionCOntrol.cpp. ADS1115 IS FINE SINCE THOSE
             WILL BE GARBAGE VALUES, BUT WE STILL TECHNICALLY SET SOME PINS TO HIGH WITH DIGITALWRITE
             SO THAT IS PROBABLY UNSAFE
             */

    /* Liquid propulsion GPIO setup (solenoid pins) */

    // Configure Solenoid ASIEthanolPin, and immidetely write HIGH to command it CLOSED on relay
    pinMode(MissionConstants::kASIEthanolPin, OUTPUT);
    digitalWrite(MissionConstants::kASIEthanolPin, 1); // HIGH = CLOSED

    // Configure Solenoid kASIOxygenPin, and immidetely write HIGH to command it CLOSED on relay
    pinMode(MissionConstants::kASIOxygenPin, OUTPUT);
    digitalWrite(MissionConstants::kASIOxygenPin, 1); // HIGH = CLOSED

    // Configure Solenoid kNitrogenBleedPin, and immidetely write HIGH
    // which in this cases turns on NitrogenBleed because it is normally OPEN which
    // is ok because we want the default state for all the solenoids
    pinMode(MissionConstants::kNitrogenBleedPin, OUTPUT);
    digitalWrite(MissionConstants::kNitrogenBleedPin, 1);

    // Spark plug pins
    // Configure kSparkPin, immidetely write HIGH to command it CLOSED on relay,
    // and send a 0% duty cycle wave to the kRPMPin
    pinMode(MissionConstants::kSparkPin, OUTPUT);
    digitalWrite(MissionConstants::kSparkPin, 1);
    pwm_driver->set_pwm(MissionConstants::kRPMPin, 0, 0); // 0% duty cycle

    //  Initialize servos to closed position (179 degrees)
    float pulse = 1500 + ((MissionConstants::kValveClosedAngle - 90) / 90.0) * 1000;
    int ticks = (pulse / MissionConstants::SERVO_PERIOD) * MissionConstants::MAX_TICKS;
    servo_driver->set_pwm(MissionConstants::kNitrogenServoPin, 0, ticks);
    servo_driver->set_pwm(MissionConstants::kPurgeServoPin, 0, ticks);
    servo_driver->set_pwm(MissionConstants::kMainEthanolServoPin, 0, ticks);
    servo_driver->set_pwm(MissionConstants::kMainNitrousServoPin, 0, ticks);

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

    while (!stopRequested && mode.Update(navigation, controller, gps, igniter, imu, magnetometer, valveControl,
                                         sparkPlug, pressureTransducer, loadCell, camera))
    {
    }
// Turn off everything now that everything is safe
#ifdef NDEBUG
    digitalWrite(MissionConstants::kASIEthanolPin, 1); // HIGH = CLOSED
    digitalWrite(MissionConstants::kASIOxygenPin, 1);  // HIGH = CLOSED
    digitalWrite(MissionConstants::kNitrogenBleedPin, 1);
    digitalWrite(MissionConstants::kSparkPin, 1);
    pwm_driver->set_pwm(MissionConstants::kRPMPin, 0, 0); // 0% duty cycle
    float pulse = 1500 + ((MissionConstants::kValveClosedAngle - 90) / 90.0) * 1000;
    int ticks = (pulse / MissionConstants::SERVO_PERIOD) * MissionConstants::MAX_TICKS;
    servo_driver->set_pwm(MissionConstants::kNitrogenServoPin, 0, ticks);
    servo_driver->set_pwm(MissionConstants::kPurgeServoPin, 0, ticks);
    servo_driver->set_pwm(MissionConstants::kMainEthanolServoPin, 0, ticks);
    servo_driver->set_pwm(MissionConstants::kMainNitrousServoPin, 0, ticks);

#endif

    return 0;
}
