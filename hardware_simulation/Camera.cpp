// hardware_simulation/Camera.cpp
#include "Camera.hpp"
#include "UDPClient.hpp"

Camera::Camera() {}

void Camera::RequestCapture()
{
    // Simulation provides camera vectors asynchronously via UDP.
}

std::tuple<double, double, double, double, double, double, double, double, double> Camera::GetUnitVectors()
{
    return UDPClient::GetInstance().GetUnitVectors();
}

std::vector<Eigen::Vector3d> Camera::GetUnitVectorList()
{
    return UDPClient::GetInstance().GetUnitVectorList();
}

double Camera::GetFrameId()
{
    return UDPClient::GetInstance().GetCameraFrameId();
}
