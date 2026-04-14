#include "ValveControl.hpp"
#include "Telemetry.hpp"
#include <wiringPi.h>
#include "ServoDriver.hpp"

void ValveControl::OpenValve(ValveType valve)
{
    float ticks = convert_to_ticks(MissionConstants::kValveOpenAngle);
    switch (valve)
    {
    case Nitrogen:
        // set_pwm_freq takes ticks related to how long high is: 4095 = 100% duty cycle
        // 1500-2500 us for 90-180 degrees
        // Forumla:
        // float pulse = 1500.0 + ((angle -90)/ 90.0) * 1000.0;
        // int ticks = (pulse / 20000.0) * 4096.0;
        Telemetry::GetInstance().Log("Opening Nitrogen Valve");
  
        servo_driver->set_pwm(MissionConstants::kNitrogenServoPin, 0, ticks);
        nitrogenOpen = true;
        break;
    case Purge:
        Telemetry::GetInstance().Log("Opening purge Valve");
        servo_driver->set_pwm(MissionConstants::kPurgeServoPin, 0, ticks);
        purgeOpen = true;
        break;
    case MainEthanol:
        Telemetry::GetInstance().Log("Opening Main Ethanol Valve");
        servo_driver->set_pwm(MissionConstants::kMainEthanolServoPin, 0, ticks);
        mainEthanolOpen = true;
        break;
    case MainNitrous:
        Telemetry::GetInstance().Log("Opening Main Nitrous Valve");
        servo_driver->set_pwm(MissionConstants::kMainNitrousServoPin, 0, ticks);
        mainNitrousOpen = true;
        break;
    case ASIEthanol:
        Telemetry::GetInstance().Log("Opening ASI Ethanol Valve");
        digitalWrite(MissionConstants::kASIEthanolPin, 0); // LOW = OPEN (normally-closed solenoid)
        asiEthanolOpen = true;
        break;
    case ASIOxygen:
        Telemetry::GetInstance().Log("Opening ASI Oxygen Valve");
        digitalWrite(MissionConstants::kASIOxygenPin, 0); // LOW = OPEN (normally-closed solenoid)
        asiOxygenOpen = true;
        break;
    case NitrogenBleed:
        Telemetry::GetInstance().Log("Opening Nitrogen Bleed Valve");
        digitalWrite(MissionConstants::kNitrogenBleedPin, 1); // HIGH = OPEN (normally-open valve)
        nitrogenBleedOpen = true;
        break;
    }
}

void ValveControl::CloseValve(ValveType valve)
{
    float ticks = convert_to_ticks(MissionConstants::kValveClosedAngle);

    switch (valve)
    {
    case Nitrogen:
        // set_pwm takes pulse width in microseconds: 500-2500 us for 0-180 degrees
        // 179 degrees = 500 + (179 * 2000 / 180) = ~510 us
        Telemetry::GetInstance().Log("Closing Nitrogen Valve");
        servo_driver->set_pwm(MissionConstants::kNitrogenServoPin, 0, ticks);
        nitrogenOpen = false;
        break;
    case Purge:
        Telemetry::GetInstance().Log("Closing purge Valve");
        servo_driver->set_pwm(MissionConstants::kPurgeServoPin, 0, ticks);
        purgeOpen = false;
        break;
    case MainEthanol:
        Telemetry::GetInstance().Log("Closing Main Ethanol Valve");
        servo_driver->set_pwm(MissionConstants::kMainEthanolServoPin, 0, ticks);
        mainEthanolOpen = false;
        break;
    case MainNitrous:
        Telemetry::GetInstance().Log("Closing Main Nitrous Valve");
        servo_driver->set_pwm(MissionConstants::kMainNitrousServoPin, 0, ticks);
        mainNitrousOpen = false;
        break;
    case ASIEthanol:
        Telemetry::GetInstance().Log("Closing Main Ethanol Valve");
        digitalWrite(MissionConstants::kASIEthanolPin, 1); // HIGH = CLOSED
        asiEthanolOpen = false;
        break;
    case ASIOxygen:
        Telemetry::GetInstance().Log("Closing ASI Oxygen Valve");
        digitalWrite(MissionConstants::kASIOxygenPin, 1); // HIGH = CLOSED
        asiOxygenOpen = false;
        break;
    case NitrogenBleed:
        Telemetry::GetInstance().Log("Closing Nitrogen Bleed Valve");
        digitalWrite(MissionConstants::kNitrogenBleedPin, 0); // LOW = CLOSED
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
