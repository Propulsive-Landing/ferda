#include <chrono>
#include <cmath>
#include "Mode.hpp"
#include "Navigation.hpp"
#include "MissionConstants.hpp"
#include "Telemetry.hpp"
#include "RF.hpp"
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

    if(seconds >= 1){
        Telemetry::GetInstance().Log("Switching mode from calibration to test");
        controller.ImportControlParameters("../k_matrix.csv");
        controller.Center();
        return Mode::TestTVC;
    }
    
    return Mode::Calibration;
}

Mode::Phase Mode::UpdateTestTVC(Navigation& navigation, Controller& controller) {

    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();

    float seconds = milliseconds_since_start / 1000.0;

    controller.UpdateTestTVC(seconds);

    if(seconds >= 10){
        Telemetry::GetInstance().Log("Switching mode from test to idle");
        controller.Center();
        return Mode::Idle;
    }
    
    return Mode::TestTVC;
}



Mode::Phase Mode::UpdateIdle(Navigation& navigation, Controller& controller, bool reset) {
    

    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();



    navigation.UpdateNavigation();
    RF::Command command = RF::GetInstance().GetCommand();

    // launch when we get the command
    if(command == RF::Command::Ignite){
        Telemetry::GetInstance().Log("Switching mode from idle to launch");
        navigation.reset();
        return Mode::Launch;

    }


    return Mode::Idle;
}

Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, Igniter& igniter, double change_time) {
   
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();

   // Launch rocket and start Controller on first iteration
    static int iteration = 1;
    if(iteration > 0){
        Telemetry::GetInstance().Log("Igniting MOTOR");
        igniter.Ignite(Igniter::IgnitionSpecifier::LAUNCH);
        controller.Start(change_time);
        iteration--;
    }

    if(milliseconds_since_start > 50){
        igniter.DisableIgnite(Igniter::IgnitionSpecifier::LAUNCH);
    }

    navigation.UpdateNavigation();
    controller.UpdateLaunch(navigation, change_time);
    

    Eigen::Matrix<double, 12, 1> testState = navigation.GetNavigation();
    // If z acceleration is negative and the z height is not the starting height, then we should go to freefall

    // COMMENTED OUT FOR STABILITY TEST
    // if(testState(5) < 0 && testState(2) > 0.28){ 
    //     std::cout<<"We are switching to freefall"<<"\n";
    //     Telemetry::GetInstance().Log("Switching mode from launch to freefall");
    //     igniter.DisableIgnite(Igniter::IgnitionSpecifier::LAUNCH);
    //     return Mode::Freefall;
    // }
    // else{
        return Mode::Launch;
    // }
}

Mode::Phase Mode::UpdateFreefall(Navigation& navigation, Igniter& igniter, double currTime) {
    
    // Continue to update navigation
    navigation.UpdateNavigation();
    //controller.UpdateLand(navigation, currTime);


    // Get currentState
    Eigen::Matrix<double, 12, 1> currentState = navigation.GetNavigation();
    
    // If the current time is greater than the calibration time + motor thrust duration + and offset, then figure out the best time to ignite
    if(currTime > MissionConstants::kFSWCalibrationTime + motor_thrust_duration + offset){
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

    // Figure out the height through the first iteration through the kinematic equations
    static double height = -9.81/2 * pow(time_till_second_ignite,2) - currentState(5)*time_till_second_ignite + currentState(2);

     // If the current height is around 20 meters, then ignite the second motor
     if (currentState(2) < height - 20){
        igniter.Ignite(Igniter::IgnitionSpecifier::LAND);
        return Mode::Land;
    }


    return Mode::Freefall;

}

Mode::Phase Mode::UpdateLand(Navigation& navigation, Controller& controller, double currTime){
   
    // Continue to update navigation and controller
    navigation.UpdateNavigation();
    controller.UpdateLand(navigation, currTime);
  
    // If the get height method returns a value between 0 and 1, then we have landed and can go to Safe.
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
    static bool liftoff = false;
    
    static int i =0;
    static double currTime = 0;    

    
    // Make sure the time between last_time and time_now is 0.005 and then keep on adding that change_time to currTime
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


    // Update navigations and controller's loopTime to be changeTime which should be 0.005 each time
    navigation.loopTime = change_time;
    controller.loopTime = change_time;

    // std::cout << std::to_string(eCurrentMode) << "\n";

    // If currTime is greater than the designated Calibrated Time, then assign true to both reset and liftoff
   if(currTime> MissionConstants::kFSWCalibrationTime && liftoff == false){
     reset = true;
     liftoff = true;
   }


    /* Handle behavior based on current phase. Update phase*/
    switch(this->eCurrentMode)
    {
        case Calibration:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05, 0.08);
            this->eCurrentMode = UpdateCalibration(navigation, controller);
            break;
        case TestTVC:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05, 0.08);
            this->eCurrentMode = UpdateTestTVC(navigation, controller);
            break;
        case Idle:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05, 0.08);
            this->eCurrentMode = UpdateIdle(navigation, controller, reset);
            break;
        case Launch:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01, 0.08);
            this->eCurrentMode = UpdateLaunch(navigation, controller, igniter, currTime);
            break;
        case Freefall:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01, 0.08);
            this->eCurrentMode = UpdateFreefall(navigation, igniter, currTime);
            break;
        case Land:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01, 0.08);
            this->eCurrentMode = UpdateLand(navigation, controller, currTime);
            break;
        case Safe:
            this->eCurrentMode = UpdateSafeMode(navigation, controller);
            break;
        case Terminate:
            return false;
    }

    return true; 

}