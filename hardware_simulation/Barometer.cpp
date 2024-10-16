#include "Barometer.hpp"
#include "UDPClient.hpp"
#include <iostream>
#include <string>

Barometer::Barometer() {}

double Barometer::GetPressure() {
    return UDPClient::GetInstance().GetPressure();
}

double Barometer::GetTemperature() {
    return UDPClient::GetInstance().GetTemperature();
}