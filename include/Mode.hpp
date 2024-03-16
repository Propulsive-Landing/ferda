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
                Launch,
                Freefall,
                Land,
                Terminate 
            };
        
        Mode(Mode::Phase eInitialMode);
        bool Update(Navigation& navigation, Controller& controller, Igniter& igniter);
    private:
        Mode::Phase eCurrentMode;

        Mode::Phase UpdateCalibration(Navigation& navigation, Controller& controller);
        Mode::Phase UpdateIdle(Navigation& navigation, Controller& controller, double current_Time, int i, bool change);
        Mode::Phase UpdateStartLaunch(Navigation& navigation, Controller& controller, double change_time);
        Mode::Phase UpdateLaunch(Navigation& navigation, Controller& controller, Igniter& igniter, double current_time, int i);
        Mode::Phase UpdateFreefall(Navigation& navigation);
        Mode::Phase UpdateSafeMode(Navigation& navigation, Controller& controller);
        Mode::Phase UpdateLand();
        Mode::Phase UpdateTestTVC(Controller& controller);

};
