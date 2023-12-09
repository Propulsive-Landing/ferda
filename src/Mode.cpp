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

Mode::Phase Mode::UpdateCalibration(Navigation& navigation, Controller& controller) {
    navigation.Start();
    controller.Center();
    Telemetry::GetInstance().SendString("Calibration is completed. Going to IDLE");
    return Mode::Idle;
}

Mode::Phase Mode::UpdateIdle(Navigation& navigation, Controller& controller) {
    navigation.UpdateNavigation();
    controller.UpdateIdle(navigation);  // Might not need this at all. It's here as a placeholder for now
    Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.1, 0.1); // The two data rates will be put in a MissonConstants file
    Telemetry::Command cmd =  Telemetry::GetInstance().GetCommand();
    if (cmd == Telemetry::Command::Startup) {
        return Mode::Launch;
    }
    return Mode::Idle;
}

Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, double change_time) {
   
   // Create a static variable to initalize the Start time so cotroller can call start the first time this method is called
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    static int iteration = 1;
    if(iteration > 0){
        controller.Start(seconds);
        iteration--;
    }
    navigation.UpdateNavigation();
    // Added chang_time so updateLaunch includes iteratng through K
    controller.UpdateLaunch(navigation, seconds);
    
    std::tuple<double,double,double> acceleration = navigation.GetBodyAcceleration();
    double acceleration_vector = (sqrt(pow(std::get<0>(acceleration),2) + pow(std::get<1>(acceleration), 2) + pow(std::get<2>(acceleration), 2)));
    if(abs(acceleration_vector) < 1){ 
        return Mode::Freefall;
    }
    else{
        return Mode::Launch;
    }
}

Mode::Phase Mode::UpdateFreefall(Navigation& navigation) {
    // some checks
    navigation.UpdateNavigation();

    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    if(seconds >= 10){//10 seconds after launch transition to safe mode and keep collecting data
        return Mode::Safe;
    }

    return Mode::Freefall;
}

Mode::Phase Mode::UpdateSafeMode(Navigation& navigation, Controller& controller){
    //continue collection data
    navigation.UpdateNavigation();
    Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.1, 0.1);

    return Mode::Safe;
}

bool Mode::Update(Navigation& navigation, Controller& controller) {
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto time_now = std::chrono::high_resolution_clock::now();
    double change_time = (time_now.time_since_epoch() - last_time.time_since_epoch()).count();
    last_time = time_now;
    /* Finish calculating time change*/


    /* Handle behavior based on current phase. Update phase*/
    switch(this->eCurrentMode)
    {
        case Calibration:
            this->eCurrentMode = UpdateCalibration(navigation, controller);
            break;
        case Idle:
            this->eCurrentMode = UpdateIdle(navigation, controller);
            break;
        case TestTVC:
            this->eCurrentMode = UpdateTestTVC(controller);
            break;
        case Launch:
            this->eCurrentMode = UpdateLaunch(navigation, controller, change_time);
            break;
        case Freefall:
            this->eCurrentMode = UpdateFreefall(navigation);
            break;
        case StartLand:
            this->eCurrentMode = UpdateStartLand();
            break;
        case Land:
            this->eCurrentMode = UpdateLand();
            break;
        case Safe:
            this->eCurrentMode = UpdateSafeMode(navigation, controller);
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
        controller.Center();
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



