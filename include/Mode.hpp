#pragma once

#include "Barometer.hpp"
#include "IMU.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"
#include "Controller.hpp"

class Mode
{
    public:
        enum Phase
            {
                Idle,
                TestTVC,
                Launch,
                Terminate 
            };
        
        Mode(Mode::Phase eInitialMode);
        bool Update(Navigation& navigation, Controller& controller);
    private:
        Mode::Phase eCurrentMode;

        Mode::Phase UpdateTestTVC(Controller& controller);
        Mode::Phase UpdateIdle(Navigation& navigation, Controller& controller);
        Mode::Phase UpdateLaunch(Navigation& navigation, Controller& controller, double change_time);
};
