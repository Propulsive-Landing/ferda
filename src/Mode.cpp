#include <chrono>

#include "Mode.hpp"
#include "Telemetry.hpp"

Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

Mode::Phase Mode::UpdateIdle(Navigation& navigation, Controller& controller, Telemetry& telemetry) {
    navigation.UpdateNavigation();
    controller.UpdateIdle(navigation);  // Might not need this at all. It's here as a placeholder for now
    telemetry.RunTelemetry(navigation, controller, 0.1, 0.1); // The two data rates will be put in a MissonConstants file
    Telemetry::Command cmd = telemetry.CheckForCommand();
    if (cmd == Telemetry::Command::Startup) {
        return Mode::StartLaunch;
    }
    return Mode::Idle;
}

Mode::Phase Mode::UpdateStartLaunch(Navigation& navigation, Controller& controller, Telemetry& telemetry, double change_time) {
    // loop for 10 seconds (this is countdown)
    static auto start_time = std::chrono::high_resolution_clock::now().count();
    while (std::chrono::high_resolution_clock::now().count() - start_time < 10000) {
        navigation.UpdateNavigation();
        telemetry.RunTelemetry(navigation, controller, 0.1, 0.1); // The two data rates will be put in a MissonConstants file
    }
    // send ignite command to launch pad
    telemetry.SendCommand(Telemetry::Command::Ignite);
    // // clamps let go (if we have clamps)
    // telemetry.SendCommand(Telemetry::Command::Release);
    return Mode::Launch;
}


Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, double change_time) {

    navigation.UpdateNavigation();
    controller.UpdateLaunch(navigation);

    // TODO Calculate next phase

    return Mode::Launch;
}


Mode::Phase Mode::UpdateFreefall(){ return Mode::Terminate; } // TODO Implement freefall phase behavior and return next phase
Mode::Phase Mode::UpdateStartLand(){ return Mode::Terminate; } // TODO Implement start-land phase behavior and return next phase
Mode::Phase Mode::UpdateLand(){ return Mode::Terminate; } // TODO Implement land phase behavior and return next phase


bool Mode::Update(Navigation& navigation, Controller& controller, Telemetry& telemetry)
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
            this->eCurrentMode = UpdateIdle(navigation, controller, telemetry);
            break;
        case StartLaunch:
            this->eCurrentMode = UpdateStartLaunch(navigation, controller, telemetry, change_time);
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

    return true; 

}