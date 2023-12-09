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
                Calibration,
                Idle,
                TestTVC,
                Launch,
                Freefall,
                StartLand,
                Land,
                Terminate,
                Safe 
            };
        
        Mode(Mode::Phase eInitialMode);
        bool Update(Navigation& navigation, Controller& controller);
    private:
        Mode::Phase eCurrentMode;

        Mode::Phase UpdateCalibration(Navigation& navigation, Controller& controller);
        Mode::Phase UpdateIdle(Navigation& navigation, Controller& controller);
        Mode::Phase UpdateStartLaunch(Navigation& navigation, Controller& controller, double change_time);
        Mode::Phase UpdateLaunch(Navigation& navigation, Controller& controller, double current_time);
        Mode::Phase UpdateFreefall(Navigation& navigation);
        Mode::Phase UpdateSafeMode(Navigation& navigation, Controller& controller);
        Mode::Phase UpdateStartLand();
        Mode::Phase UpdateLand();
        

};
