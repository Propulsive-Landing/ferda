#include <chrono>

#include "Mode.hpp"


Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

Mode::Phase Mode::UpdateIdle(){ return Mode::Launch; } // TODO Implement idle state behavior


Mode::Phase Mode::UpdateLaunch(Navigation navigation, Controller controller, double change_time){

    navigation.UpdateNavigation();
    controller.updateLaunch(navigation);

    // TODO Calculate next stage

    return Mode::Land;
}

Mode::Phase Mode::UpdateLand(){ return Mode::Terminate; } // TODO Implement land state behavior


bool Mode::Update(Navigation& navigation, Controller& controller)
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
            this->eCurrentMode = UpdateLaunch(navigation, controller, change_time);
            break;
        case Land:
            this->eCurrentMode = UpdateLand();
            break;
        case Terminate:
            return false;
    }

    return true;
}