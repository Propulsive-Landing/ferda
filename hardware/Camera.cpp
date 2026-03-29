// TODO: Update camera class for hardware implementaiton

// hardware_simulation/Camera.cpp
#include "Camera.hpp"

Camera::Camera() {}

std::tuple<double, double, double, double, double, double, double, double, double> Camera::GetUnitVectors()
{
    return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}
double Camera::GetFrameId()
{
    return -1.0;
}
