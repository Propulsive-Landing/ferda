#include "TVC.hpp"

#include <iostream>
#include <string>

void TVC::AnglesToActuatorLengths(double angle_x_rad, double angle_y_rad,
                                  double &length_x, double &length_y)
{
}

void TVC::ProportionalPositionControl(int actuator_index)
{
}

void TVC::SetTVCX(double dAngle)
{
    dAngle += 90 + MissionConstants::kTvcYCenterAngleDeg;
    dAngle = (dAngle < 0) ? 0 : dAngle;
    dAngle = (dAngle > 180) ? 180 : dAngle;

    double dPulseWidth = 1000 + (dAngle * 1000 / 180.0);
    std::cout << "Wrote angle to X: " + std::to_string(dAngle) + " PW: " + std::to_string(dPulseWidth) + "\n";
}

void TVC::SetTVCY(double dAngle)
{
    dAngle += 90 + MissionConstants::kTvcYCenterAngleDeg;
    dAngle = (dAngle < 0) ? 0 : dAngle;
    dAngle = (dAngle > 180) ? 180 : dAngle;

    double dPulseWidth = 1000 + (dAngle * 1000 / 180.0);

    std::cout << "Wrote angle to Y: " + std::to_string(dAngle) + " PW: " + std::to_string(dPulseWidth) + "\n";
}

void TVC::UpdateActuatorPositions()
{
}