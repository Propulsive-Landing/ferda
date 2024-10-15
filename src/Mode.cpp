#include <chrono>
#include <cmath>
#include "Mode.hpp"
#include "Navigation.hpp"
#include "MissionConstants.hpp"
#include "Telemetry.hpp"
#include "RF.hpp"
#include <iostream>
#include <sstream>
#include <string>
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
double second_motor_delta_x = 59.1075;
double gse_height = 0.2800;
double result = 0;
double time_till_second_ignite = 0;

Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

Mode::Phase Mode::UpdateCalibration(Navigation& navigation, Controller& controller, double currentTime) {
    static float XTVC = 0.0;
    static float YTVC = 0.0;

    RF::Command command = RF::GetInstance().GetCommand();
    if(command == RF::Command::IncrementXTVC){
        XTVC += 0.1;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCX(XTVC);
        return Mode::Calibration;
    }
    if(command == RF::Command::IncrementYTVC){
        YTVC += 0.1;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCY(XTVC);
        return Mode::Calibration;
    }
    
    if(command == RF::Command::DecrementXTVC){
        XTVC -= 0.1;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCX(XTVC);
        return Mode::Calibration;
    }
    if(command == RF::Command::DecrementYTVC){
        YTVC -= 0.1;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCY(XTVC);
        return Mode::Calibration;
    }

    if(command == RF::Command::TestTVC){
        Telemetry::GetInstance().Log("Switching mode from calibration to test tvc");
        controller.ImportControlParameters("../k_matrix.csv");
        controller.Center();
        return Mode::TestTVC;
    }
    else if(command == RF::Command::GoIdle){
        Telemetry::GetInstance().Log("Switching mode from calibration to idle");
        controller.ImportControlParameters("../k_matrix.csv");
        controller.Center();
        return Mode::Idle;
    }
    else if(command == RF::Command::ABORT){
        Telemetry::GetInstance().Log("ABORT, EXITING");
	exit(0);
    }
    
    return Mode::Calibration;
}

Mode::Phase Mode::UpdateTestTVC(Navigation& navigation, Controller& controller, double currentTime) {


    static double startTime = currentTime;
    double seconds_since_start = currentTime - startTime;
    
    controller.UpdateTestTVC(seconds_since_start);

    RF::Command command = RF::GetInstance().GetCommand();
    if(command == RF::Command::ABORT){
        Telemetry::GetInstance().Log("ABORT, EXITING");
	exit(0);
    }
    else if(seconds_since_start >= 10){
        Telemetry::GetInstance().Log("Switching mode from test to idle");
        controller.Center();
        return Mode::Idle;
    }
    
    return Mode::TestTVC;
}



Mode::Phase Mode::UpdateIdle(Navigation& navigation, Controller& controller, double currentTime) {


    navigation.UpdateNavigation();
    
    // launch when we get the command
    RF::Command command = RF::GetInstance().GetCommand();
    if(command == RF::Command::ABORT){
        Telemetry::GetInstance().Log("ABORT, EXITING");
	    exit(0);
    }
    else if(command == RF::Command::Ignite){
        Telemetry::GetInstance().Log("Switching mode from idle to launch");
        navigation.reset();
        return Mode::Launch;

    }


    return Mode::Idle;
}

Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, Igniter& igniter, double currentTime) {
   // Launch rocket and start Controller on first iteration
    static double startTime = currentTime;
    double seconds_since_start = currentTime - startTime;

    static int startup = 1;
    if(startup == 1){
        Telemetry::GetInstance().Log("Igniting MOTOR");
        igniter.Ignite(Igniter::IgnitionSpecifier::LAUNCH);
        controller.Start(seconds_since_start);
        startup = 0;
    }

    if(seconds_since_start > 0.050){
        igniter.DisableIgnite(Igniter::IgnitionSpecifier::LAUNCH);
    }

    navigation.UpdateNavigation();
    controller.UpdateLaunch(navigation, seconds_since_start);
    

    Eigen::Matrix<double, 12, 1> testState = navigation.GetNavigation();
    // If z acceleration is negative and the z height is not the starting height, then we should go to freefall

    // COMMENTED OUT FOR STABILITY TEST
    if(testState(5) < 0 && testState(2) > 0.28){ 
         std::cout<<"We are switching to freefall"<<"\n";
         Telemetry::GetInstance().Log("Switching mode from launch to freefall");
         igniter.DisableIgnite(Igniter::IgnitionSpecifier::LAUNCH);
         return Mode::Freefall;
     }
     else{
        return Mode::Launch;
     }
}

Mode::Phase Mode::UpdateFreefall(Navigation& navigation, Controller &controller, Igniter& igniter, double currentTime) {
    
    // Continue to update navigation
    navigation.UpdateNavigation();
    //controller.UpdateLand(navigation, currTime);


    // Get currentState
    Eigen::Matrix<double, 12, 1> currentState = navigation.GetNavigation();
    
    // If the current time is greater than the calibration time + motor thrust duration + and offset, then figure out the best time to ignite
    // TODO THIS LOGIC IS BAD, CURRENT TIME IS VARIABLE DEPENDING ON LAUNCH PROCEDURE
    double a = -9.81/2;
    double b = currentState(5) + (-9.81*(motor_thrust_duration * motor_thrust_percentage));

    double average_landing_throttle = 1;

    double c = currentState(5) * (motor_thrust_duration * motor_thrust_percentage) + currentState(2) + -9.81*0.5*pow((motor_thrust_duration*motor_thrust_percentage),2) + average_landing_throttle*second_motor_delta_x - gse_height;

    result = (-b - sqrt(pow(b,2) - 4*a*c)) / (2*a);

    time_till_second_ignite = result;

    if (time_till_second_ignite < 0){
        igniter.Ignite(Igniter::IgnitionSpecifier::LAND);
        std::cout<< "Switching from Freefall to Land" << "\n";
        controller.ResetKIteration(currentTime);
        return Mode::Land;
    }

    return Mode::Freefall;

}

Mode::Phase Mode::UpdateLand(Navigation& navigation, Controller& controller, double currentTime){
   
    // Continue to update navigation and controller
    navigation.UpdateNavigation();
    controller.UpdateLand(navigation, currentTime);
  
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

Mode::Phase Mode::UpdateSafeMode(Navigation& navigation, Controller& controller, double currentTime){
    //continue collection data
    navigation.UpdateNavigation();

    return Mode::Terminate;
}


bool Mode::Update(Navigation& navigation, Controller& controller, Igniter& igniter) { 

    // Track total elapsed time and delta time
    static double currentTime = 0;    
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto time_now = std::chrono::high_resolution_clock::now();
    unsigned int nanoseconds_since_start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - last_time).count();
    double change_time = nanoseconds_since_start / 1000000000.0;
    last_time = time_now;
    
    currentTime += change_time;


    // Update navigations and controller's loopTime to be changeTime which should be 0.005 each time
    navigation.loopTime = change_time;
    controller.loopTime = change_time;


    /* Handle behavior based on current phase. Update phase*/
    switch(this->eCurrentMode)
    {
        case Calibration:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05, 0.08);
            this->eCurrentMode = UpdateCalibration(navigation, controller, currentTime);
            break;
        case TestTVC:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05, 0.08);
            this->eCurrentMode = UpdateTestTVC(navigation, controller, currentTime);
            break;
        case Idle:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05, 0.08);
            this->eCurrentMode = UpdateIdle(navigation, controller, currentTime);
            break;
        case Launch:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01, 0.08);
            this->eCurrentMode = UpdateLaunch(navigation, controller, igniter, currentTime);
            break;
        case Freefall:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01, 0.08);
            this->eCurrentMode = UpdateFreefall(navigation, controller, igniter, currentTime);
            break;
        case Land:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01, 0.08);
            this->eCurrentMode = UpdateLand(navigation, controller, currentTime);
            break;
        case Safe:
            this->eCurrentMode = UpdateSafeMode(navigation, controller, currentTime);
            break;
        case Terminate:
            return false;
    }

    return true; 

}
