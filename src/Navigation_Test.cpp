#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "Navigation.hpp"
#include "Barometer.hpp"
#include "TVC.hpp"
#include "IMU.hpp"
#include "MissionConstants.hpp"
#include <cmath>


class Navigation_Test : public ::testing::Test{};
 
// Create Barometer, TVC, IMU objects so we can initalize a testing Navigation object
Barometer b = Barometer();
TVC t = TVC();
IMU i = IMU();
Navigation s = Navigation(i,b,t);

/*
TEST(Navigation_Test, RotationalMatrixHappyPath) {
    double phi = M_PI;
    double theta = phi/2;
    double psi = 0;
    
    Eigen::Matrix3d res;
      res<<  0, 0, -1,
            0, -1, 0,
            -1, 0, 0;

    ASSERT_EQ(s.CreateRotationalMatrix(phi, theta, psi), res);
}
*/
TEST(Navigation_Test, D_Theta_Now_Math){
    std::vector<double> theta_vel = {1,0,0};
    std::tuple<double,double,double> AngularAcceleration = {1,0,0};
    double phi = M_PI;
    double theta = phi/2;
    double psi = 0;

    ASSERT_EQ(s.D_Theta_Now_Math(phi, theta, psi, AngularAcceleration), theta_vel);
}

TEST(Navigation_Test, GetNavigation){
    // Test GetNavigation() 
    Eigen::Matrix<double, 12, 1> State = Eigen::Matrix<double, 12, 1>::Zero();
    ASSERT_EQ(s.GetNavigation(), State);
}

TEST(Navigation_Test, getImuAceleration){
    // Test GetBodyAcceleration
    std::tuple<double,double,double> Acceleration = {0,0,10};
    ASSERT_EQ(s.GetBodyAcceleration(), Acceleration);
}
/*
TEST(Navigation_Test, updateNavigation){
    // Tests updateNavigation
    
    State << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    s.UpdateNavigation();
    ASSERT_EQ(s.GetNavigation(), State);
    
}
*/
int main() {
    // Initialize Google Test framework
    ::testing::InitGoogleTest();
    // Run all tests
   return RUN_ALL_TESTS();
};
