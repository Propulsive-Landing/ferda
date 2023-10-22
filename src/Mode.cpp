#include <chrono>

#include "Mode.hpp"


Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

Mode::Phase Mode::UpdateIdle() {
    // TODO: Implement idle state behavior
    // TODO: calculate next stage
    return Mode::Launch;
}


Mode::Phase Mode::UpdateLaunch(Navigation navigation, Controller controller, double change_time) {

    navigation.UpdateNavigation();
    controller.UpdateLaunch(navigation);

    // TODO Calculate next stage
    // check for aborts
    //  if we're too far slanted, set throttle paddles to max
    // if acceleration == -9.81, return Mode::Freefall

    return Mode::Land;
}

Mode::Phase Mode::UpdateFreefall() {
    // some checks
    navigation.UpdateNavigation();

    // if angle is too far, abort
    // x[4]
    // stateMat = [x, y, z, xdot, ydot, zdot, phi, theta, psi, phidot, thetadot, psidot]
}

Mode::Phase Mode::UpdateLand() {
    return Mode::Terminate;
} // TODO Implement land state behavior


bool Mode::Update(Navigation& navigation, Controller& controller) {
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