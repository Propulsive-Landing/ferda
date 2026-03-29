// hardware_simulation/Camera.cpp
#include "Camera.hpp"
#include "UDPClient.hpp"

Camera::Camera() {}

std::tuple<double, double, double, double, double, double, double, double, double> Camera::GetUnitVectors()
{
    return UDPClient::GetInstance().GetUnitVectors();
}
double Camera::GetFrameId()
{
    return UDPClient::GetInstance().GetCameraFrameId();
}
