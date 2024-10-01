// hardware_simulation/TVC.cpp
#include "TVC.hpp"
#include "UDPClient.hpp"

void TVC::SetXServo(double dAngle) {
    UDPClient::GetInstance().SetTVCX(dAngle);
}

void TVC::SetYServo(double dAngle) {
    UDPClient::GetInstance().SetTVCY(dAngle);
}