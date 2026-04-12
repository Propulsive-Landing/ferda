// hardware_simulation/Camera.cpp
#include "Camera.hpp"
#include "UDPClient.hpp"

Camera::Camera() {}

void Camera::RequestCapture()
{
    // Simulation provides camera vectors asynchronously via UDP.
}

std::vector<Eigen::Vector3d> Camera::GetUnitVectorList()
{
    return UDPClient::GetInstance().GetUnitVectorList();
}

double Camera::GetFrameId()
{
    return UDPClient::GetInstance().GetCameraFrameId();
}

void Camera::AnnotateDebugFrameMatches(
    double frameId,
    const std::vector<std::pair<int, int>>& measurementToMarkerMatches)
{
    (void)frameId;
    (void)measurementToMarkerMatches;
}
