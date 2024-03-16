#include "TVC.hpp"

#include <iostream>
#include <string>
#include "MissionConstants.hpp"

void TVC::SetXServo(double dAngle)
{
   // dAngle = MissionConstants::kDeg2Rad * dAngle;
    dAngle += 90 + MissionConstants::kTvcXCenterAngle;
    dAngle = (dAngle < 0) ? 0 : dAngle;
    dAngle = (dAngle > 180) ? 180 : dAngle;

    double dPulseWidth = 1000 + (dAngle * 1000 / 180.0);
   // std::cout << "Wrote: " << std::to_string(dAngle) << " to x servo\n";
    return;
}

void TVC::SetYServo(double dAngle)
{
   // dAngle = MissionConstants::kDeg2Rad * dAngle;
    dAngle += 90 + MissionConstants::kTvcYCenterAngle;
    dAngle = (dAngle < 0) ? 0 : dAngle;
    dAngle = (dAngle > 180) ? 180 : dAngle;

    double dPulseWidth = 1000 + (dAngle * 1000 / 180.0);
   // std::cout << "Wrote: " << std::to_string(dAngle) << " to y servo\n";
    return;
}