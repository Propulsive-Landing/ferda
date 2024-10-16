// hardware_simulation/TVC.cpp
#include "TVC.hpp"
#include "UDPClient.hpp"

void TVC::SetTVCX(double angle_rad) {
    UDPClient::GetInstance().SetTVCX(angle_rad);
}

void TVC::SetTVCY(double angle_rad) {

    UDPClient::GetInstance().SetTVCY(angle_rad);
}