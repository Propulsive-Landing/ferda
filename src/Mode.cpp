#include <chrono>

#include "Mode.hpp"


Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

Mode::Phase Mode::UpdateIdle(){ return Mode::Launch; } // TODO Implement idle phase behavior and return next phase
Mode::Phase Mode::UpdateStartLaunch(){ return Mode::Terminate; } // TODO Implement start-land phase behavior and return next phase


Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, double change_time){

    navigation.UpdateNavigation();
    controller.UpdateLaunch(navigation);

    // TODO Calculate next phase

    return Mode::Land;
}


Mode::Phase Mode::UpdateFreefall(){ return Mode::Terminate; } // TODO Implement freefall phase behavior and return next phase
Mode::Phase Mode::UpdateStartLand(){ return Mode::Terminate; } // TODO Implement start-land phase behavior and return next phase
Mode::Phase Mode::UpdateLand(){ return Mode::Terminate; } // TODO Implement land phase behavior and return next phase


bool Mode::Update(Navigation& navigation, Controller& controller)
{
    /* Start calculate time change*/
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto time_now = std::chrono::high_resolution_clock::now();
    double change_time = (time_now.time_since_epoch() - last_time.time_since_epoch()).count();
    last_time = time_now;
    /* End calculate time change*/


    /* Handle behavior based on current phase. Update phase*/
    switch(this->eCurrentMode)
    {
        case Idle:
            this->eCurrentMode = UpdateIdle();
            break;
        case StartLaunch:
            this->eCurrentMode = UpdateStartLaunch();
            break;
        case Launch:
            this->eCurrentMode = UpdateLaunch(navigation, controller, change_time);
            break;
        case Freefall:
            this->eCurrentMode = UpdateFreefall();
            break;
        case StartLand:
            this->eCurrentMode = UpdateStartLand();
            break;
        case Land:
            this->eCurrentMode = UpdateLand();
            break;
        case Terminate:
            return false;
    }

    return false; // State number is not handled if this is reached
}