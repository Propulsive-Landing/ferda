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
        navigation.Start();
        controller.ImportControlParameters("../k_matrix.csv");
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

Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, Igniter& igniter, double change_time) {
   
   // Launch rocket
    igniter.Ignite(Igniter::IgnitionSpecifier::LAUNCH);

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
        return Mode::Freefall;
    }
    else{
        return Mode::Launch;
    }
}

Mode::Phase Mode::UpdateFreefall(Navigation& navigation, Igniter& igniter) {
    // some checks

    /*
       *SIMULATION CODE*
    if (time > fsw_calibration_time + motor_thrust_duration + offset) && ~estimated
    %this math finds at what time from now the change in position caused by the rocket
    %starting its second motor is equal to the rocket's current height
    %above 0.28 m (its COM).
    %Detailed description of the math can be found here: https://www.desmos.com/calculator/rc7gpykiil
    a = -9.81/2;
    b = xhat(6)+(-9.81*(motor_thrust_duration*motor_thrust_percentage));
    
    %Clamp time on the pad will lead to throttle being needed on the way
    %down. This calculates the average ammount of throttle for the descent
    %and adjusts the expected motor change in position accordingly
    average_landing_throttle = 1-(interp1(x_series(:,1),x_series(:,3),fsw_clamp_time)/interp1(x_series(:,1),x_series(:,3),motor_thrust_percentage*motor_thrust_duration));
    if fsw_throttle_descent == 0
        average_landing_throttle = 1;
    end

    c = xhat(6)*(motor_thrust_duration*motor_thrust_percentage) + xhat(3) + -9.81*0.5*(motor_thrust_duration*motor_thrust_percentage)^2 + average_landing_throttle*second_motor_delta_x - gse_height;

    result = (-b - sqrt((b^2)-(4*a*c)))/(2*a);
    estimated = true;
   end
    */
    navigation.UpdateNavigation();
    Eigen::Matrix<double, 12, 1> currentState = navigation.GetNavigation();


    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    if(seconds > MissionConstants::kFswLoopTime + motor_thrust_duration + offset){
       double a = -9.81/2;
       double b = currentState(5) + (-9.81*(motor_thrust_duration+motor_thrust_percentage));

       double average_landing_throttle = 1;

       double c = currentState(5) * (motor_thrust_duration * motor_thrust_percentage) + currentState(2) + -9.81*0.5*pow((motor_thrust_duration*motor_thrust_percentage),2) + average_landing_throttle*second_motor_delta_x - gse_height;

       result = (-b - sqrt(pow(b,2) - 4*a*c)) / (2*a);

       time_till_second_ignite = result + offset;
    }

    if (time_till_second_ignite > MissionConstants::kFswLoopTime + motor_thrust_duration + offset + result){
        igniter.Ignite(Igniter::IgnitionSpecifier::LAND);
        return Mode::Land;

    }

    // WRITE CODE TO GO TO LAND
    return Mode::Freefall;
}

Mode::Phase Mode::UpdateLand(Navigation& navigation, Controller& controller, double change_time){
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    navigation.UpdateNavigation();

    std::tuple<double,double,double> acceleration = navigation.GetBodyAcceleration();
    double acceleration_vector = (sqrt(pow(std::get<0>(acceleration),2) + pow(std::get<1>(acceleration), 2) + pow(std::get<2>(acceleration), 2)));
    if ( 9.7 < acceleration_vector && acceleration_vector < 9.9 && 0.0 < navigation.GetHeight() && navigation.GetHeight() < 1.0)
        return Mode::Safe;

    return Mode::Land;
}

Mode::Phase Mode::UpdateSafeMode(Navigation& navigation, Controller& controller){
    //continue collection data
    navigation.UpdateNavigation();

    return Mode::Safe;
}


bool Mode::Update(Navigation& navigation, Controller& controller, Igniter& igniter) {
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
        case Launch:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateLaunch(navigation, controller, igniter, change_time);
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
        case Terminate:
            return false;
    }

    return true; 

}