#include <chrono>
#include <cmath>
#include "Mode.hpp"
#include "Navigation.hpp"
#include "Telemetry.hpp"
#include <iostream>


//CONSTANTS TO BE FIGURED OUT LATER
int abort_threshold = 1;
int calibration_time = 1;
int thrust_duration = 1;
int descent_time = 1;
int total_time = 1;
double ignition_height = 1;

Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

Mode::Phase Mode::UpdateCalibration(Navigation& navigation, Controller& controller) {
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;
    static bool centered = false;

    if(seconds >= 1){
       // Telemetry::GetInstance().Log("Switching mode from calibration to idle");
        navigation.Start();
        controller.ImportControlParameters("../12-9-k-matrix.csv");
        return Mode::Idle;

    }
    
    return Mode::Calibration;
}

Mode::Phase Mode::UpdateIdle(Navigation& navigation, Controller& controller) {
    navigation.UpdateNavigation();

    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    if(seconds > 1){
        Telemetry::GetInstance().Log("Switching mode from idle to launch");
        static auto start_time = std::chrono::high_resolution_clock::now();
        return Mode::Launch;
    }


    return Mode::Idle;
}

Mode::Phase Mode::UpdateStartLaunch(Navigation& navigation, Controller& controller, double change_time){
     static auto start_time = std::chrono::high_resolution_clock::now();
     int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
     double seconds = milliseconds_since_start / 1000.0;
     return Mode::Launch;
/*
     if (seconds >= 10){
        launch
     }
}
*/
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
        Telemetry::GetInstance().Log("Switching mode from launch to freefall");
       // if(seconds > navigation.loopTime + motor_thrust_duration)
       //     ejct parachute
        return Mode::Freefall;
    }
    else{
        return Mode::Launch;
    }
}

Mode::Phase Mode::UpdateFreefall(Navigation& navigation) {
    // some checks
  //  if 0 > xhat(6) && time > fsw_calibration_time + motor_thrust_duration

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

    return Mode::Safe;
}


Mode::Phase Mode::UpdateTestTVC(Controller& controller) {
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;
    
    controller.UpdateTestTVC(seconds);

    if(seconds > 5.0){
        start_time = std::chrono::high_resolution_clock::now();
        controller.Center();
        return Mode::Calibration;
    }

    return Mode::TestTVC;
}


bool Mode::Update(Navigation& navigation, Controller& controller) {
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto time_now = std::chrono::high_resolution_clock::now();
    double change_time = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - last_time).count() / 1000000000.0;
    last_time = time_now;
    /* Finish calculating time change*/

    navigation.loopTime = change_time;
    controller.loopTime = change_time;

    std::cout << std::to_string(eCurrentMode) << "\n";


    /* Handle behavior based on current phase. Update phase*/
    switch(this->eCurrentMode)
    {
        case Calibration:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05);
            this->eCurrentMode = UpdateCalibration(navigation, controller);
            break;
        case Idle:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05);
            this->eCurrentMode = UpdateIdle(navigation, controller);
            break;
        case TestTVC:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateTestTVC(controller);
            break;
        case StartLaunch:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateStartLaunch(navigation, controller, change_time);
            break;
        case Launch:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateLaunch(navigation, controller, change_time);
            break;
        case Freefall:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateFreefall(navigation);
            break;
        case Safe:
            this->eCurrentMode = UpdateSafeMode(navigation, controller);
        case Terminate:
            return false;
    }

    return true; 

}