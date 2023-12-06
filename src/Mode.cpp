#include <chrono>
#include <cmath>
#include "Mode.hpp"
#include "Navigation.hpp"

//CONSTANTS TO BE FIGURED OUT LATER
int abort_threshold = 1;
int calibration_time = 1;
int thrust_duration = 1;
int descent_time = 1;
int total_time = 1;
double ignition_height = 1;

Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}


bool Mode::Update(Navigation& navigation, Controller& controller) {
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto time_now = std::chrono::high_resolution_clock::now();
    double change_time = (time_now.time_since_epoch() - last_time.time_since_epoch()).count();
    last_time = time_now;
    /* Finish calculating time change*/


    /* Handle behavior based on current phase. Update phase*/
    switch(this->eCurrentMode)
    {
        case Idle:
            this->eCurrentMode = UpdateIdle(navigation, controller);
            break;
        case TestTVC:
            this->eCurrentMode = UpdateTestTVC(controller);
            break;
        case Launch:
            this->eCurrentMode = UpdateLaunch(navigation, controller, change_time);
            break;
        case Terminate:
            return false;
    }

    return true; 

}


Mode::Phase Mode::UpdateIdle(Navigation& navigation, Controller& controller) {
    navigation.UpdateNavigation();
    controller.UpdateIdle(navigation);  // Might not need this at all. It's here as a placeholder for now


    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    if(seconds > 5.0){
        static auto start_time = std::chrono::high_resolution_clock::now();
        return Mode::Launch;
    }


    return Mode::Idle;
}

Mode::Phase Mode::UpdateTestTVC(Controller& controller) {
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;
    
    controller.UpdateTestTVC(seconds);

    if(seconds > 5.0){
        static auto start_time = std::chrono::high_resolution_clock::now();
        return Mode::Idle;
    }

    return Mode::TestTVC;
}


Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, double change_time) {

    navigation.UpdateNavigation();
    controller.UpdateLaunch(navigation);

    // TODO Calculate next phase
    return Mode::Launch;
}



