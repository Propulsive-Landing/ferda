#include "State.hpp"

State::State(Mode eInitialMode) : eCurrentMode(eInitialMode) {}

State::Mode State::UpdateIdle(){}
State::Mode State::UpdateLaunch(){}
State::Mode State::UpdateLand(){}


bool State::Update()
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