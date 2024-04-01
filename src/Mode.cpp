#include <chrono>
#include <cmath>
#include "Mode.hpp"
#include "Navigation.hpp"
#include "MissionConstants.hpp"
#include "Telemetry.hpp"
#include <iostream>

//CONSTANTS TO BE FIGURED OUT LATER
int abort_threshold = 1;
int calibration_time = 1;
int descent_time = 1;
int total_time = 1;
double ignition_height = 1;
double offset = 0.45;
double motor_thrust_duration = 2.4250;
double motor_thrust_percentage = 1;
double fsw_clamp_time = 0.300;
double second_motor_delta_x = 99.6451;
double gse_height = 0.2800;
double result = 0;
double time_till_second_ignite = 0;

Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

Mode::Phase Mode::UpdateCalibration(Navigation& navigation, Controller& controller) {
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;
    static bool centered = false;

    if(seconds >= 1){
       // Telemetry::GetInstance().Log("Switching mode from calibration to idle");
        controller.ImportControlParameters("../k_matrix.csv");
        controller.Center();
        return Mode::Idle;
    }
    
    return Mode::Calibration;
}


Mode::Phase Mode::UpdateIdle(Navigation& navigation, Controller& controller, bool reset) {
    navigation.UpdateNavigation();

    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    if(reset){
        Telemetry::GetInstance().Log("Switching mode from idle to launch");
        static auto start_time = std::chrono::high_resolution_clock::now();
        navigation.reset();
        return Mode::Launch;
    }


    return Mode::Idle;
}

Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, Igniter& igniter, double change_time) {
   
   // Launch rocket and start Controller on first iteration
    static int iteration = 1;
    if(iteration > 0){
        igniter.Ignite(Igniter::IgnitionSpecifier::LAUNCH);
        controller.Start(change_time);
        iteration--;
    }

    navigation.UpdateNavigation();
    // Added chang_time so updateLaunch includes iteratng through K
    controller.UpdateLaunch(navigation, change_time);
    

    Eigen::Matrix<double, 12, 1> testState = navigation.GetNavigation();

    if(testState(5) < 0 && testState(2) > 0.28){ 
        std::cout<<"We are switching to freefall"<<"\n";
        Telemetry::GetInstance().Log("Switching mode from launch to freefall");
        return Mode::Freefall;
    }
    else{
        return Mode::Launch;
    }
}

Mode::Phase Mode::UpdateFreefall(Navigation& navigation, Igniter& igniter, double currTime) {
    // some checks

    
    navigation.UpdateNavigation();
    //controller.UpdateLand(navigation, currTime);

    Eigen::Matrix<double, 12, 1> currentState = navigation.GetNavigation();

    if(currTime > MissionConstants::kFswLoopTime + motor_thrust_duration + offset){
       double a = -9.81/2;
       double b = currentState(5) + (-9.81*(motor_thrust_duration+motor_thrust_percentage));

       double average_landing_throttle = 1;

       double c = currentState(5) * (motor_thrust_duration * motor_thrust_percentage) + currentState(2) + -9.81*0.5*pow((motor_thrust_duration*motor_thrust_percentage),2) + average_landing_throttle*second_motor_delta_x - gse_height;

       result = (-b - sqrt(pow(b,2) - 4*a*c)) / (2*a);

       time_till_second_ignite = result + offset;
    }
    else
    {
        return Mode::Freefall;
    }

    static double height = -9.81/2 * pow(time_till_second_ignite,2) - currentState(5)*time_till_second_ignite + currentState(2);

     if (currentState(2) < height - 20){
        igniter.Ignite(Igniter::IgnitionSpecifier::LAND);
        return Mode::Land;
    }


    return Mode::Freefall;
}

Mode::Phase Mode::UpdateLand(Navigation& navigation, Controller& controller, double change_time){
   
    navigation.UpdateNavigation();
    controller.UpdateLand(navigation, change_time);

    if (0.0 < navigation.GetHeight() && navigation.GetHeight() < 1.0)
     {
         return Mode::Safe;
     }
     else
     {
        return Mode::Land;
     }
 

     return Mode::Land;
}

Mode::Phase Mode::UpdateSafeMode(Navigation& navigation, Controller& controller){
    //continue collection data
    navigation.UpdateNavigation();

    return Mode::Terminate;
}


bool Mode::Update(Navigation& navigation, Controller& controller, Igniter& igniter) {
    bool reset = false;
    bool liftoff = false;
    
    static int i =0;
    static double currTime = 0;    

    
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto time_now = std::chrono::high_resolution_clock::now();
    double change_time = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - last_time).count() / 1000.0;
    last_time = time_now;
    while(change_time < 0.005)
    {
             change_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_time).count() / 1000.0;

    }
    
    if(change_time != 0.005)
    {
        change_time = 0.005;
    }
    currTime += change_time;
    navigation.loopTime = change_time;
    controller.loopTime = change_time;

    std::cout << std::to_string(eCurrentMode) << "\n";

   if(currTime> MissionConstants::kFswCalibrationTime && liftoff == false){
     reset = true;
     liftoff = true;
   }


    /* Handle behavior based on current phase. Update phase*/
    switch(this->eCurrentMode)
    {
        case Calibration:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05);
            this->eCurrentMode = UpdateCalibration(navigation, controller);
            break;
        case Idle:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05);
            this->eCurrentMode = UpdateIdle(navigation, controller, reset);
            break;
        case Launch:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateLaunch(navigation, controller, igniter, currTime);
            break;
        case Freefall:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateFreefall(navigation, igniter);
            break;
        case Land:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateLand(navigation, controller, change_time);
            break;
        case Safe:
            this->eCurrentMode = UpdateSafeMode(navigation, controller);
            break;
        case Terminate:
            return false;
    }

    return true; 

}