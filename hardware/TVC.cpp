#include "TVC.hpp"

#include <Eigen/Dense>

#include <pigpio.h>
#include <math.h>
#include <MissionConstants.hpp>


void TVC::SetXServo(double dAngle)
{
    dAngle += 90 + MissionConstants::kTvcYCenterAngle;
    dAngle = (dAngle < 0) ? 0 : dAngle;
    dAngle = (dAngle > 180) ? 180 : dAngle;

    double dPulseWidth = 1000 + (dAngle * 1000 / 180.0);
    gpioServo(23, round(dPulseWidth));
}



void TVC::SetYServo(double dAngle)
{
    dAngle += 90 + MissionConstants::kTvcYCenterAngle;
    dAngle = (dAngle < 0) ? 0 : dAngle;
    dAngle = (dAngle > 180) ? 180 : dAngle;

    double dPulseWidth = 1000 + (dAngle * 1000 / 180.0);
    gpioServo(24, round(dPulseWidth));
}
