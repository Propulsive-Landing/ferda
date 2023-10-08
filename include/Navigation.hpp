#pragma once

#include <Eigen/Dense>

class Navigation
{
    public:
        Navigation() = default;
        Eigen::Matrix<double, 3, 1> GetNav();
};