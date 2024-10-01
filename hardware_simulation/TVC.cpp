// hardware_simulation/TVC.cpp
#include "TVC.hpp"
#include "UDPClient.hpp"

void TVC::SetTVCX(double dAngle) {
    UDPClient::GetInstance().SetTVCX(dAngle);
}

void TVC::SetTVCY(double dAngle) {
    UDPClient::GetInstance().SetTVCY(dAngle);
}