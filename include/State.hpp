#pragma once

#include "Barometer.hpp"
#include "IMU.hpp"
#include "Igniter.hpp"
#include "TVC.hpp"

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
        bool Update(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter); 
    private:
        State::Mode eCurrentMode;


        State::Mode UpdateIdle();
        State::Mode UpdateLaunch();
        State::Mode UpdateLand();

};