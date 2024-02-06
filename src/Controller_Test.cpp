#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "Navigation.hpp"
#include "Barometer.hpp"
#include "TVC.hpp"
#include "IMU.hpp"
#include "MissionConstants.hpp"
#include <cmath>

class Controller_Test : public ::testing::Test{};

// Create Barometer, TVC, IMU objects so we can initalize a testing Navigation object
Barometer b = Barometer();
TVC t = TVC();
IMU i = IMU();
Navigation s = Navigation(i,b,t);
Controller c = Controller(t)

