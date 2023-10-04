#include "State.hpp"

State::State(Mode eInitialMode) : eCurrentMode(eInitialMode) {}

Mode State::UpdateIdle();
Mode State::UpdateLaunch();
Mode State::UpdateLand();


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
}