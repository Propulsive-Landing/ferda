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
    const std::vector<std::pair<int, int>> &measurementToMarkerMatches)
{
    (void)frameId;
    (void)measurementToMarkerMatches;
}

void Camera::AnnotateDebugFrameExpectedVsTrue(
    double frameId,
    const std::vector<std::pair<int, Eigen::Vector3d>> &expectedMarkerBodyDirections)
{
    (void)frameId;
    (void)expectedMarkerBodyDirections;
}
