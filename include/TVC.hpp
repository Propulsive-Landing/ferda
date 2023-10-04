#pragma once

class TVC
{
    public:
        IMU() = default;
        
        void SetXServo(double dAngle); 
        void SetYServo(double dAngle); 
};