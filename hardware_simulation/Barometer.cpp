#include "Barometer.hpp"
#include "UDPClient.hpp"

Barometer::Barometer() {}

double Barometer::GetPressure() {
    return UDPClient::GetInstance().GetPressure();
}

double Barometer::GetTemperature() {
    return UDPClient::GetInstance().GetTemperature();
}