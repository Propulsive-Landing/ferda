#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "Navigation.hpp"


TEST(NavigationTest, RotationalMatrixHappyPath) {
    double phi = 3.141592653589793238462643383;
    double theta = phi/2;
    double psi = 0;
    Barometer b = Barometer();
    TVC t = TVC();
    IMU i = IMU();
    Navigation s = Navigation(i,b,t);


    Eigen::Matrix3d res;
      res<<  0, 0, -1,
            0, -1, 0,
            -1, 0, 0;
    

    ASSERT_EQ(s.CreateRotationalMatrix(phi, theta, psi), res);
}
