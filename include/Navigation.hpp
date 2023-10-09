#pragma once

#include <Eigen/Dense>

class Navigation
{
    public:
        Navigation() = default;
        Eigen::Matrix<double, 3, 1> GetNavigation(); // Defintion of navigation matrix: TODO (determine dimensions and document form)
        void UpdateNavigation(Eigen::Matrix<double, 4, 1> updates); // Defintion updates: TODO (determine dimensions and document form)
};