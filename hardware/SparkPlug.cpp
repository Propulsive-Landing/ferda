#include "SparkPlug.hpp"
#include <pigpio.h>

void SparkPlug::TurnOn()
{
    Telemetry::GetInstance().Log("Turning Spark on");
    gpioWrite(MissionConstants::kSparkPin, 0); // LOW = ON (relay control)
    // Set PWM to 2% duty cycle (5/255 for 8-bit PWM, but using 0-100 range)
    // For pigpio, we need to use gpioPWM which takes 0-255, so 2% = ~5
    gpioPWM(MissionConstants::kRPMPin, 5); // 2% duty cycle
    isOn = true;
}

void SparkPlug::TurnOff()
{
    Telemetry::GetInstance().Log("Turning Spark off");
    gpioWrite(MissionConstants::kSparkPin, 1); // HIGH = OFF
    gpioPWM(MissionConstants::kRPMPin, 0);     // 0% duty cycle
    isOn = false;
}

bool SparkPlug::IsOn() const
{
    return isOn;
}
