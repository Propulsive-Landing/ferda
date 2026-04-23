#include "LinActMotorPositionControl.hpp"

#include "Telemetry.hpp"

float readPositionInches(int actuator_index)
{
    (void)actuator_index;
    return 0.0f;
}

float readPositionInches()
{
    return readPositionInches(0);
}

void driveActuator(int actuator_index, int direction, int speed)
{
    (void)actuator_index;
    (void)direction;
    (void)speed;
}

int moveToLimit(int actuator_index, int direction)
{
    (void)actuator_index;
    (void)direction;
    return 0;
}

void RunChirpTVCMode()
{
    Telemetry::GetInstance().Log("Simulation mode: RunChirpTVCMode is a no-op");
}
