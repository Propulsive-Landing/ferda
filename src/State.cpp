#include <chrono>

#include "State.hpp"


State::State(Mode eInitialMode) : eCurrentMode(eInitialMode) {}

State::Mode State::UpdateIdle(){ return State::Launch; } // TODO Implement idle state behavior
State::Mode State::UpdateLaunch(){ return State::Land; } // TODO Implement launch state behavior
State::Mode State::UpdateLand(){ return State::Terminate; } // TODO Implement land state behavior


bool State::Update(Navigation& navigation, Controller& controller)
{
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto time_now = std::chrono::high_resolution_clock::now();
    double change_time = (time_now.time_since_epoch() - last_time.time_since_epoch()).count();
    last_time = time_now;


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