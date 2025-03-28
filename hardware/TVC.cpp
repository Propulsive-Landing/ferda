#include "TVC.hpp"

#include <Eigen/Dense>

#include <pigpio.h>
#include <math.h>
#include <algorithm.h>
#include <MissionConstants.hpp>
#include <iostream>
#include <string>

void TVC::SetTVCX(double angle_rad)
{
    double degrees = (MissionConstants::kTvcXInputCenterAngleRad + angle_rad) * MissionConstants::kRad2Deg;
    degees = std::clamp(degrees, -10.0, 10.0);  // Clamp to stay in linearized range.

    double servoAngle = -.000095801*powf(degrees, 4) - .0027781*powf(degrees, 3) + .0012874*powf(degrees, 2) - 3.1271*degrees -16.129;
    servoAngle += 90 + MissionConstants::kTvcXCenterAngleDeg;

    double dPulseWidth = 1000 + (servoAngle * 1000 / 180.0);
    gpioServo(23, round(dPulseWidth));
}

void TVC::SetTVCY(double angle_rad)
{
    double degrees = (MissionConstants::kTvcYInputCenterAngleRad + angle_rad) * MissionConstants::kRad2Deg;
    degees = std::clamp(degrees, -10.0, 10.0);
    
    double servoAngle = - .0002314576*powf(degrees, 4) - .002425139*powf(degrees, 3) - .01204116*powf(degrees, 2) - 2.959760*degrees + 57.18794;
    servoAngle += 90 + MissionConstants::kTvcYCenterAngleDeg;

    double dPulseWidth = 1000 + (servoAngle * 1000 / 180.0);
    gpioServo(24, round(dPulseWidth));
}
