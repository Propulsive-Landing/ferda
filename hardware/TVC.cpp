#include "TVC.hpp"

#include <pigpio.h>
#include <math.h>
#include <MissionConstants.hpp>

void TVC::SetXServo(double dAngle)
{
    dAngle += 90 + MissionConstants::kTvcXCenterAngle;
    dAngle = (dAngle < 0) ? 0 : dAngle;
    dAngle = (dAngle > 180) ? 180 : dAngle;

    double dPulseWidth = 1000 + (dAngle * 1000 / 180.0);
    gpioServo(18, round(dPulseWidth));
}

void TVC::SetYServo(double dAngle)
{
    dAngle += 90 + MissionConstants::kTvcYCenterAngle;
    dAngle = (dAngle < 0) ? 0 : dAngle;
    dAngle = (dAngle > 180) ? 180 : dAngle;

    double dPulseWidth = 1000 + (dAngle * 1000 / 180.0);
    gpioServo(13, round(dPulseWidth));
}
