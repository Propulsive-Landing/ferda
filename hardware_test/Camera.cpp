// TODO: Update camera class for hardware implementaiton

// hardware_test/Camera.cpp
#include "Camera.hpp"

Camera::Camera() {}

void Camera::RequestCapture()
{
}

std::tuple<double, double, double, double, double, double, double, double, double> Camera::GetUnitVectors()
{
    return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

std::vector<Eigen::Vector3d> Camera::GetUnitVectorList()
{
    return {};
}

double Camera::GetFrameId()
{
    return -1.0;
}
