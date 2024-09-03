#pragma once

#include "Barometer.hpp"
#include "IMU.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"
#include "Controller.hpp"
#include "Igniter.hpp"

class Mode
{
    public:
        enum Phase
            {
                Calibration,
                TestTVC,
                Idle,
                Launch,
                Freefall,
                Terminate,
                Safe,
                Land
            };
        
        Mode(Mode::Phase eInitialMode);
        bool Update(Navigation& navigation, Controller& controller, Igniter& igniter);
    private:
        Mode::Phase eCurrentMode;

        Mode::Phase UpdateCalibration(Navigation& navigation, Controller& controller);

        Mode::Phase UpdateTestTVC(Navigation& navigation, Controller& controller);

        Mode::Phase UpdateIdle(Navigation& navigation, Controller& controller, bool reset);
        Mode::Phase UpdateLaunch(Navigation& navigation, Controller& controller, Igniter& igniter, double current_time);
        Mode::Phase UpdateFreefall(Navigation& navigation, Igniter& igniter, double currTime);
        Mode::Phase UpdateLand(Navigation& navigation, Controller& controller, double current_time);
        Mode::Phase UpdateSafeMode(Navigation& navigation, Controller& controller);
};
