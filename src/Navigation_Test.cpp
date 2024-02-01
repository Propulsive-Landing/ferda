#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "Navigation.hpp"

TEST(NavigationTest, RotationalMatrixHappyPath) {
    double phi = 3.141592653589793238462643383;
    double theta = phi/2;
    double psi = 0;

    Eigen::Matrix3d res = {
        {0, 0, -1},
        {0, -1, 0},
        {-1, 0, 0}
    };

    ASSERT(Navigation::CreateRotationalMatrix(phi, theta, psi), res);
}