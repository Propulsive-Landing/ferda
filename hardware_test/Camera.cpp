// TODO: Update camera class for hardware implementaiton

// hardware_test/Camera.cpp
#include "Camera.hpp"

Camera::Camera() {}

std::tuple<double, double, double, double, double, double, double, double, double> Camera::GetUnitVectors()
{
    return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}
std::tuple<double> Camera::CameraAvailable()
{
    return 0.0;
}
