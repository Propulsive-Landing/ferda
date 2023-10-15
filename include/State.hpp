#pragma once

#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"

#include "Navigation.hpp"
#include "Controller.hpp"

class State
{
    public:
        enum Mode
            {
                Idle,
                Launch,
                Land,
                Terminate 
            };
        
        State(State::Mode eInitialMode);
        bool Update(Navigation& navigation, Controller& controller);
    private:
        State::Mode eCurrentMode;


        State::Mode UpdateIdle();
        State::Mode UpdateLaunch();
        State::Mode UpdateLand();

};