#include "TVC.hpp"

#include <iostream>
#include <string>

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

    std::cout << "Input angle: " << std::to_string(dAngle) << " Wrote angle: " << std::to_string(inputAngle) << " Width:" << std::to_string(dPulseWidth) << " to x servo\n";
    return;
}

void TVC::SetYServo(double dAngle)
{
    std::cout << "Wrote: " << std::to_string(dAngle) << " to y servo\n";
    return;
}