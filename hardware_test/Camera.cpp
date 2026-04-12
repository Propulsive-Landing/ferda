// TODO: Update camera class for hardware implementaiton

// hardware_test/Camera.cpp
#include "Camera.hpp"

Camera::Camera() {}

void Camera::RequestCapture()
{
}

std::vector<Eigen::Vector3d> Camera::GetUnitVectorList()
{
    return {};
}

double Camera::GetFrameId()
{
    return -1.0;
}

void Camera::AnnotateDebugFrameMatches(
    double frameId,
    const std::vector<std::pair<int, int>>& measurementToMarkerMatches)
{
    (void)frameId;
    (void)measurementToMarkerMatches;
}
