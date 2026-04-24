#include "SparkPlug.hpp"
#include "Telemetry.hpp"
#include <wiringPi.h>
#include <PCA9685Driver.hpp>

void SparkPlug::TurnOn()
{
    Telemetry::GetInstance().Log("Turning Spark on");
    digitalWrite(MissionConstants::kSparkPin, 0); // LOW = ON (relay control)
    // Set PWM to 2% duty cycle (servo driver which is PCA9685 is 12 bit - 1 so it goes 0-4095
    // We do 4095*0.02 = 81.9

    // NOTE: We do not handle if servo_driver is a null pointer meaning the PCA9685 is not connected
    pwm_driver->set_pwm(MissionConstants::kRPMPin, 0, 81); // 2% duty cycle
    isOn = true;
}

void SparkPlug::TurnOff()
{
    Telemetry::GetInstance().Log("Turning Spark off");
    digitalWrite(MissionConstants::kSparkPin, 1); // HIGH = OFF

    // NOTE: We do not handle if servo_driver is a null pointer meaning the PCA9685 is not connected
    pwm_driver->set_pwm(MissionConstants::kRPMPin, 0, 0); // 0% duty cycle
    isOn = false;
}

bool SparkPlug::IsOn() const
{
    return isOn;
}
