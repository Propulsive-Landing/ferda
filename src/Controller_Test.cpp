#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "Navigation.hpp"
#include "Barometer.hpp"
#include "TVC.hpp"
#include "IMU.hpp"
#include "MissionConstants.hpp"
#include "Controller.hpp"
#include <cmath>

class Controller_Test : public ::testing::Test{};

//mock classes

// Create Barometer, TVC, IMU objects so we can initalize a testing Navigation object
Barometer b = Barometer();
TVC t = TVC();
IMU i = IMU();
Navigation s = Navigation(i,b,t);
Controller c = Controller(t);


TEST(Controller_Test, TVCMath){
    Eigen::Vector2d input(0,0);
    Eigen::Vector2d result(-16.129, 57.18794);

    ASSERT_EQ(c.TvcMath(input), result);
}
int main(){
    ::testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}