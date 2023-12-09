#include "TVC.hpp"

#include <iostream>
#include <string>

void TVC::SetXServo(double dAngle)
{
    std::cout << "Wrote: " << std::to_string(dAngle) << " to x servo\n";
    return;
}

void TVC::SetYServo(double dAngle)
{
    std::cout << "Wrote: " << std::to_string(dAngle) << " to y servo\n";
    return;
}