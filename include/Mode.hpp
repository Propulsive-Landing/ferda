#pragma once

#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"
#include "Navigation.hpp"
#include "Controller.hpp"

class Mode
{
    public:
        enum Phase
            {
                Idle,
                StartLaunch,
                Launch,
                Freefall,
                StartLand,
                Land,
                Terminate 
            };
        
        Mode(Mode::Phase eInitialMode);
        bool Update(Navigation& navigation, Controller& controller, Telemetry& telemetry);
    private:
        Mode::Phase eCurrentMode;


        Mode::Phase UpdateIdle(Navigation& navigation, Controller& controller, Telemetry& telemetry);
        Mode::Phase UpdateStartLaunch(Navigation& navigation, Controller& controller, Telemetry& telemetry, double change_time);
        Mode::Phase UpdateLaunch(Navigation& navigation, Controller& controller, double change_time);
        Mode::Phase UpdateFreefall();
        Mode::Phase UpdateStartLand();
        Mode::Phase UpdateLand();

};