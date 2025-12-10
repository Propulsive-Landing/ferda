#include "ValveControl.hpp"
#include <pigpio.h>

void ValveControl::OpenValve(ValveType valve)
{
    switch (valve)
    {
    case Nitrogen:
        // gpioServo takes pulse width in microseconds: 1000-2000 us for 0-180 degrees
        // 91 degrees = 1000 + (91 * 1000 / 180) = ~1506 us
        gpioServo(MissionConstants::kNitrogenServoPin, 1000 + (MissionConstants::kValveOpenAngle * 1000 / 180));
        nitrogenOpen = true;
        break;
    case Purge:
        gpioServo(MissionConstants::kPurgeServoPin, 1000 + (MissionConstants::kValveOpenAngle * 1000 / 180));
        purgeOpen = true;
        break;
    case MainEthanol:
        gpioServo(MissionConstants::kMainEthanolServoPin, 1000 + (MissionConstants::kValveOpenAngle * 1000 / 180));
        mainEthanolOpen = true;
        break;
    case MainNitrous:
        gpioServo(MissionConstants::kMainNitrousServoPin, 1000 + (MissionConstants::kValveOpenAngle * 1000 / 180));
        mainNitrousOpen = true;
        break;
    case ASIEthanol:
        gpioWrite(MissionConstants::kASIEthanolPin, 0); // LOW = OPEN (normally-closed solenoid)
        asiEthanolOpen = true;
        break;
    case ASIOxygen:
        gpioWrite(MissionConstants::kASIOxygenPin, 0); // LOW = OPEN (normally-closed solenoid)
        asiOxygenOpen = true;
        break;
    case NitrogenBleed:
        gpioWrite(MissionConstants::kNitrogenBleedPin, 1); // HIGH = OPEN (normally-open valve)
        nitrogenBleedOpen = true;
        break;
    }
}

void ValveControl::CloseValve(ValveType valve)
{
    switch (valve)
    {
    case Nitrogen:
        // gpioServo takes pulse width in microseconds: 1000-2000 us for 0-180 degrees
        // 179 degrees = 1000 + (179 * 1000 / 180) = ~1994 us
        gpioServo(MissionConstants::kNitrogenServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
        nitrogenOpen = false;
        break;
    case Purge:
        gpioServo(MissionConstants::kPurgeServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
        purgeOpen = false;
        break;
    case MainEthanol:
        gpioServo(MissionConstants::kMainEthanolServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
        mainEthanolOpen = false;
        break;
    case MainNitrous:
        gpioServo(MissionConstants::kMainNitrousServoPin, 1000 + (MissionConstants::kValveClosedAngle * 1000 / 180));
        mainNitrousOpen = false;
        break;
    case ASIEthanol:
        gpioWrite(MissionConstants::kASIEthanolPin, 1); // HIGH = CLOSED
        asiEthanolOpen = false;
        break;
    case ASIOxygen:
        gpioWrite(MissionConstants::kASIOxygenPin, 1); // HIGH = CLOSED
        asiOxygenOpen = false;
        break;
    case NitrogenBleed:
        gpioWrite(MissionConstants::kNitrogenBleedPin, 0); // LOW = CLOSED
        nitrogenBleedOpen = false;
        break;
    }
}

bool ValveControl::IsValveOpen(ValveType valve) const
{
    switch (valve)
    {
    case Nitrogen:
        return nitrogenOpen;
    case Purge:
        return purgeOpen;
    case MainEthanol:
        return mainEthanolOpen;
    case MainNitrous:
        return mainNitrousOpen;
    case ASIEthanol:
        return asiEthanolOpen;
    case ASIOxygen:
        return asiOxygenOpen;
    case NitrogenBleed:
        return nitrogenBleedOpen;
    default:
        return false;
    }
}

