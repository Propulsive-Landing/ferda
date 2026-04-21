#include <iostream>
#include <fstream>
#include <wiringPi.h>
#include <signal.h>
#include <cmath>
#include <mutex>
#include <ads1115.h>
#include <PiPCA9685/PCA9685.h>
#include "MissionConstants.hpp"
#include "LinActMotorPositionControl.hpp"

#define ADS_BASE 100
#define SENSOR_PIN_0 (ADS_BASE + 0)           // PLACEHOLDER: X-axis actuator sensor
#define SENSOR_PIN_1 (ADS_BASE + 1)           // PLACEHOLDER: Y-axis actuator sensor

PiPCA9685::PCA9685 pca;

namespace
{
std::once_flag gActuatorHardwareInitOnce;

void InitializeActuatorHardware()
{
    pca.set_pwm_freq(1000); // Set frequency to 1000 Hz for motor control

    if (ads1115Setup(ADS_BASE, 0x48) < 0)
    {
        std::cerr << "WARNING: ads1115Setup failed for actuator position sensing" << std::endl;
    }
}

void EnsureActuatorHardwareInitialized()
{
    std::call_once(gActuatorHardwareInitOnce, InitializeActuatorHardware);
}
}


float extensionLength;

int maxReading = MissionConstants::kTvcActuator0PotentiometerMaxReading;
int minReading = MissionConstants::kTvcActuator0PotentiometerMinReading;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

float readPositionInches(int actuator_index)
{
    EnsureActuatorHardwareInitialized();

    int sensorPin = (actuator_index == 0) ? SENSOR_PIN_0 : SENSOR_PIN_1;
    const int actuatorMinReading = (actuator_index == 0)
        ? MissionConstants::kTvcActuator0PotentiometerMinReading
        : MissionConstants::kTvcActuator1PotentiometerMinReading;
    const int actuatorMaxReading = (actuator_index == 0)
        ? MissionConstants::kTvcActuator0PotentiometerMaxReading
        : MissionConstants::kTvcActuator1PotentiometerMaxReading;
    
    int sensorVal = analogRead(sensorPin);
    return mapFloat(
        (float)sensorVal,
        (float)actuatorMinReading,
        (float)actuatorMaxReading,
        0.0f,
        MissionConstants::kTvcStrokeLengthInches
    );
}

// Legacy function for backward compatibility (defaults to actuator 0)
float readPositionInches()
{
    return readPositionInches(0);
}

// ----------------------
// Motor Control
// ----------------------
void driveActuator(int actuator_index, int direction, int speed)
{
    EnsureActuatorHardwareInitialized();

    int rpwmChannel = MissionConstants::kTvcActuator0RpwmChannel;
    int lpwmChannel = MissionConstants::kTvcActuator0LpwmChannel;

    if (actuator_index == 1)
    {
        rpwmChannel = MissionConstants::kTvcActuator1RpwmChannel;
        lpwmChannel = MissionConstants::kTvcActuator1LpwmChannel;
    }

    switch(direction)
    {
        case 1: // extend
            
            std::cout << "Driving actuator " << actuator_index << " to extend at speed " << speed << std::endl;
        
            pca.set_pwm(rpwmChannel, 0, speed);
            pca.set_pwm(lpwmChannel, 0, 0);
            break;

        case 0: // stop

            std::cout << "Stopping actuator " << actuator_index << " to extend at speed " << speed << std::endl;

            pca.set_pwm(rpwmChannel, 0, 0);
            pca.set_pwm(lpwmChannel, 0, 0);
            break;

        case -1: // retract
            std::cout << "Retracting actuator " << actuator_index << " to retract at speed " << speed << std::endl;
            pca.set_pwm(rpwmChannel, 0, 0);
            pca.set_pwm(lpwmChannel, 0, speed);
            break;
    }
}

// ----------------------
// Linear chirp velocity command
// v(t) = Vmax * sin(2*pi*(f0*t + 0.5*k*t^2))
// ----------------------
void applyLinearChirpVelocityCommand(float durationSec,
                                     float startFreqHz,
                                     float endFreqHz,
                                     int maxSpeed,
                                     int controlPeriodMs = 20)
{
    if (durationSec <= 0.0f || controlPeriodMs <= 0)
    {
        std::cout << "Invalid chirp timing parameters\n";
        return;
    }

    if (maxSpeed < 0) maxSpeed = 0;
    if (maxSpeed > 4095) maxSpeed = 4095;

    std::ofstream logFile("chirp_velocity_log.csv", std::ios::app);
    if (!logFile)
    {
        std::cout << "Failed to open chirp_velocity_log.csv\n";
        return;
    }
    if (logFile.tellp() == 0)
    {
        logFile << "time,velocity_command,position\n";
    }

    const float pi = 3.14159265358979323846f;
    const float k = (endFreqHz - startFreqHz) / durationSec;
    const unsigned int startMs = millis();

    while (true)
    {
        float t = (millis() - startMs) / 1000.0f;
        if (t >= durationSec)
        {
            break;
        }

        float phase = 2.0f * pi *
                      (startFreqHz * t + 0.5f * k * t * t);
        float velocityCmd = std::sin(phase); // normalized [-1, 1]

        int direction = 0;
        if (velocityCmd > 0.0f)
        {
            direction = 1;
        }
        else if (velocityCmd < 0.0f)
        {
            direction = -1;
        }

        int speedCmd = static_cast<int>(std::abs(velocityCmd) * maxSpeed);
        driveActuator(0, direction, speedCmd);

        float position = readPositionInches(0);
        logFile << t << "," << velocityCmd << "," << position << "\n";

        delay(controlPeriodMs);
    }

    driveActuator(0, 0, 0);
}

void RunChirpTVCMode()
{
    applyLinearChirpVelocityCommand(MissionConstants::kTvcChirpDurationSec,
                                    MissionConstants::kTvcChirpStartFreqHz,
                                    MissionConstants::kTvcChirpEndFreqHz,
                                    MissionConstants::kTvcChirpMaxSpeed,
                                    MissionConstants::kTvcChirpControlPeriodMs);
}

// ----------------------
// Velocity step command
// Applies a constant signed velocity command for a fixed duration.
// stepAmplitude should be in [-1, 1].
// ----------------------
void applyVelocityStepCommand(float durationSec,
                              float stepAmplitude,
                              int maxSpeed,
                              int controlPeriodMs = 20)
{
    if (durationSec <= 0.0f || controlPeriodMs <= 0)
    {
        std::cout << "Invalid step timing parameters\n";
        return;
    }

    if (maxSpeed < 0) maxSpeed = 0;
    if (maxSpeed > 4095) maxSpeed = 4095;

    std::ofstream logFile("step_velocity_log.csv", std::ios::app);
    if (!logFile)
    {
        std::cout << "Failed to open step_velocity_log.csv\n";
        return;
    }
    if (logFile.tellp() == 0)
    {
        logFile << "time,velocity_command,position\n";
    }

    if (stepAmplitude > 1.0f) stepAmplitude = 1.0f;
    if (stepAmplitude < -1.0f) stepAmplitude = -1.0f;

    int direction = 0;
    if (stepAmplitude > 0.0f)
    {
        direction = 1;
    }
    else if (stepAmplitude < 0.0f)
    {
        direction = -1;
    }

    int speedCmd = static_cast<int>(std::abs(stepAmplitude) * maxSpeed);

    const unsigned int startMs = millis();
    while (true)
    {
        float t = (millis() - startMs) / 1000.0f;
        if (t >= durationSec)
        {
            break;
        }

        driveActuator(0, direction, speedCmd);
        float position = readPositionInches(0);
        logFile << t << "," << stepAmplitude << "," << position << "\n";
        delay(controlPeriodMs);
    }

    driveActuator(0, 0, 0);
}

// ----------------------
// Move to limit (auto-calibration)
// ----------------------
int moveToLimit(int actuator_index, int direction)
{
    int prev = 0;
    int curr = 0;
    int sensorPin = (actuator_index == 0) ? SENSOR_PIN_0 : SENSOR_PIN_1;

    do
    {
        prev = curr;

        driveActuator(actuator_index, direction, MissionConstants::kTvcMaxMotorSpeed);
        delay(200);

        curr = analogRead(sensorPin);

        float voltage = (curr / 32767.0) * 6.144;
        std::cout << "Actuator " << actuator_index << " Raw: " << curr << " Voltage: " << voltage << "\n";

    } while (abs(curr - prev) > 10); // tolerance for noise

    driveActuator(actuator_index, 0, 0);
    return curr;
}

// ----------------------
// Float mapping
// ----------------------
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}