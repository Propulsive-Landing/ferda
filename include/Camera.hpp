#pragma once

#include <tuple>

class Camera
{
public:
    Camera();
    // Returns the four unit vectors corresponding to the directions of the markers in the camera frame
    std::tuple<double, double, double, double, double, double, double, double, double> GetUnitVectors();
    double GetFrameId();
};