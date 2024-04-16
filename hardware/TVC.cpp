#include "TVC.hpp"

#include <Eigen/Dense>

#include <pigpio.h>
#include <math.h>
#include <MissionConstants.hpp>

void TVC::SetXServo(double dAngle)
{
    /* trim */
    double commanded = dAngle;
    if (abs(commanded) > MissionConstants::kMaximumTvcAngleDeg)
    {
        commanded *= MissionConstants::kMaximumTvcAngleDeg / abs(commanded);
    }



    Eigen::Matrix<double, 2, 1> input_vector = (Eigen::MatrixXd(2,1) << 
                        dAngle, 
                        0.0).finished();



    Eigen::Matrix<double, 2, 1> u_servo = TVC_GEAR_RATIO * input_vector + TVC_SERVO_HORN_OFFSET;
    double servo_x = u_servo(0, 0);
    

    double inputAngle = servo_x;
    inputAngle = (inputAngle < 0) ? 0 : inputAngle;
    inputAngle = (inputAngle > 180) ? 180 : inputAngle;

    double dPulseWidth = 1000 + (inputAngle * 1000 / 180.0);
    gpioServo(23, round(dPulseWidth));
}

void TVC::SetYServo(double dAngle)
{
    /* trim */
    double commanded = dAngle;
    if (abs(commanded) > MissionConstants::kMaximumTvcAngleDeg)
    {
        commanded *= MissionConstants::kMaximumTvcAngleDeg / abs(commanded);
    }



    Eigen::Matrix<double, 2, 1> input_vector = (Eigen::MatrixXd(2,1) << 
                        0.0, 
                        dAngle).finished();



    Eigen::Matrix<double, 2, 1> u_servo = TVC_GEAR_RATIO * input_vector + TVC_SERVO_HORN_OFFSET;
    double servo_y = u_servo(1, 0);
    

    double inputAngle = servo_y;
    inputAngle = (inputAngle < 0) ? 0 : inputAngle;
    inputAngle = (inputAngle > 180) ? 180 : inputAngle;

    double dPulseWidth = 1000 + (inputAngle * 1000 / 180.0);
    gpioServo(24, round(dPulseWidth));
}


// void TVC::SetYServo(double dAngle)
// {
//     dAngle += 90 + MissionConstants::kTvcYCenterAngle;
//     dAngle = (dAngle < 0) ? 0 : dAngle;
//     dAngle = (dAngle > 180) ? 180 : dAngle;

//     double dPulseWidth = 1000 + (dAngle * 1000 / 180.0);
//     gpioServo(24, round(dPulseWidth));
// }
