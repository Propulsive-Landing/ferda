#pragma once

#include <tuple>

class TVC
{
    public:
        IMU() = default;
        
        void SetXServo(double dAngle); 
        void SetYServo(double dAngle); 
};