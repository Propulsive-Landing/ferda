#include "TVC.hpp"

#include <Eigen/Dense>

#include <pigpio.h>
#include <math.h>
#include <MissionConstants.hpp>


void TVC::SetTVCX(double angle_rad)
{
    double degrees = angle_rad * MissionConstants::kRad2Deg;
    double servoAngle = -.000095801*powf(degrees, 4) - .0027781*powf(degrees, 3) + .0012874*powf(degrees, 2) - 3.1271*degrees -16.129;

    servoAngle += 90 + MissionConstants::kTvcXCenterAngle;
    servoAngle = (servoAngle < 0) ? 0 : servoAngle;
    servoAngle = (servoAngle > 180) ? 180 : servoAngle;

    double dPulseWidth = 1000 + (servoAngle * 1000 / 180.0);
    gpioServo(16, round(dPulseWidth));
}

void TVC::SetTVCY(double angle_rad)
{

    double degrees = angle_rad * MissionConstants::kRad2Deg;
    double servoAngle = - .0002314576*powf(degrees, 4) - .002425139*powf(degrees, 3) - .01204116*powf(degrees, 2) - 2.959760*degrees + 57.18794;


    servoAngle += 90 + MissionConstants::kTvcYCenterAngle;
    servoAngle = (servoAngle < 0) ? 0 : servoAngle;
    servoAngle = (servoAngle > 180) ? 180 : servoAngle;

    double dPulseWidth = 1000 + (servoAngle * 1000 / 180.0);
    gpioServo(18, round(dPulseWidth));
}
