#include <iostream>
#include <fstream>
#include <signal.h>
#include <cmath>
#include <mutex>
#include "MissionConstants.hpp"
#include "LinActMotorPositionControl.hpp"

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

float readPositionInches(int actuator_index)
{
    return 0;
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
    int rpwmChannel = MissionConstants::kTvcActuator0RpwmChannel;
    int lpwmChannel = MissionConstants::kTvcActuator0LpwmChannel;

    if (actuator_index == 1)
    {
        rpwmChannel = MissionConstants::kTvcActuator1RpwmChannel;
        lpwmChannel = MissionConstants::kTvcActuator1LpwmChannel;
    }

    switch (direction)
    {
    case 1: // extend

        std::cout << "Driving actuator " << actuator_index << " to extend at speed " << speed << std::endl;

        break;

    case 0: // stop

        std::cout << "Stopping actuator " << actuator_index << " to extend at speed " << speed << std::endl;

        break;

    case -1: // retract
        std::cout << "Retracting actuator " << actuator_index << " to retract at speed " << speed << std::endl;
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
}

// ----------------------
// Move to limit (auto-calibration)
// ----------------------
int moveToLimit(int actuator_index, int direction)
{
    return 0;
}

// ----------------------
// Float mapping
// ----------------------
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}