#include "State.hpp"

State::State(Mode eInitialMode) : eCurrentMode(eInitialMode) {}

State::Mode State::UpdateIdle(){ return State::Launch; } // TODO Implement idle state behavior
State::Mode State::UpdateLaunch(){ return State::Land; } // TODO Implement launch state behavior
State::Mode State::UpdateLand(){ return State::Terminate; } // TODO Implement land state behavior


bool State::Update(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter)
{
    switch(this->eCurrentMode)
    {
        case Idle:
            this->eCurrentMode = UpdateIdle();
            break;
        case Launch:
            this->eCurrentMode = UpdateLaunch();
            break;
        case Land:
            this->eCurrentMode = UpdateLand();
            break;
        case Terminate:
            return false;
    }

    return true;
}