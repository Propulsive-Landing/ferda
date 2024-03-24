#include <chrono>
#include <cmath>
#include "Mode.hpp"
#include "Navigation.hpp"
#include "Telemetry.hpp"
#include <iostream>
#include <vector>
#include "MissionConstants.hpp"


//CONSTANTS TO BE FIGURED OUT LATER
int abort_threshold = 1;
int calibration_time = 1;
int thrust_duration = 1;
int descent_time = 1;
int total_time = 1;
double ignition_height = 1;
double motor_thrust_duration = 2.09;
double motor_thrust_percentage = 1;
double fsw_clamp_time = 0.300;
double second_motor_delta_x = 40.6855;
double gse_height = 0.2800;
double result = 0;
double time_till_second_ignite = 0;
double offset = 0.45;

static std::vector<std::vector<double>> testStates;
static std::vector<std::vector<double>> testControllerStates;
static std::vector<std::vector<double>> testInputs;


Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

Mode::Phase Mode::UpdateCalibration(Navigation& navigation, Controller& controller) {
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;
    static bool centered = false;

  //  if(seconds >= 1){
       // Telemetry::GetInstance().Log("Switching mode from calibration to idle");
        navigation.importTestAccAndTestGyro();
        navigation.importTestBarom();
        controller.ImportControlParameters("../12-9-k-matrix.csv");
        controller.Center();
        return Mode::Idle;
 //   }
    
    return Mode::Calibration;
}

Mode::Phase Mode::UpdateIdle(Navigation& navigation, Controller& controller,Igniter& igniter, double current_time, int i, bool check) {

     Eigen::Matrix<double, 12, 1> testState;
     Eigen::Matrix<double, 8, 1> testControllerState;
     Eigen::Vector2d testInput;
     std::vector<double> testStateVector;
     std::vector<double> testControllerStateVector;
     std::vector<double> testInputsVector;


    double time = current_time;
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    if(check == true)
    {
        Telemetry::GetInstance().Log("Switching mode from idle to launch");
        navigation.reset();

    
        testState = navigation.GetNavigation();
        testControllerState = controller.getXControl();
        testInput = controller.getInput();
        // std::cout<<testState<<"\n";

        testStateVector.push_back(current_time);
        testControllerStateVector.push_back(current_time);
        testInputsVector.push_back(current_time);
        
        for(int j = 1; j <= 12; j++)
        {
            testStateVector.push_back(testState(j-1));
        } 
        testStates.push_back(testStateVector);
        
        for(int k = 1; k <=8; k++)
        {
            testControllerStateVector.push_back(testControllerState(k-1));
        }
        testControllerStates.push_back(testControllerStateVector);
        
        for(int l = 1; l <= 2; l++)
        {
            testInputsVector.push_back(testInput(l-1));
        }
        testInputs.push_back(testInputsVector);
        
        std::cout<<i<<"\n";
        igniter.Ignite(Igniter::IgnitionSpecifier::LAUNCH);

        return Mode::Launch;
    }

     navigation.UpdateNavigation(i);
    
    testState = navigation.GetNavigation();
    testControllerState = controller.getXControl();
    testInput = controller.getInput();
    // std::cout<<testState<<"\n";

    
    testStateVector.push_back(current_time);
    testControllerStateVector.push_back(current_time);
    testInputsVector.push_back(current_time);
        
    for(int j = 1; j <= 12; j++)
    {
        testStateVector.push_back(testState(j-1));
    } 
    testStates.push_back(testStateVector);
        
    for(int k = 1; k <=8; k++)
    {
        testControllerStateVector.push_back(testControllerState(k-1));
    }
    testControllerStates.push_back(testControllerStateVector);
        
    for(int l = 1; l <= 2; l++)
    {
        testInputsVector.push_back(testInput(l-1));
    }
    testInputs.push_back(testInputsVector);
    return Mode::Idle;
}

Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, double curr_time, int i) {
 

    static int iteration = 1;
    if(iteration > 0){
        controller.Start(curr_time);
        iteration--;
    }

     Eigen::Matrix<double, 12, 1> testState;
     Eigen::Matrix<double, 8, 1> testControllerState;
     Eigen::Vector2d testInput;
     std::vector<double> testStateVector;
     std::vector<double> testControllerStateVector;
     std::vector<double> testInputsVector;

     double time = curr_time;

   // Create a static variable to initalize the Start time so cotroller can call start the first time this method is called   
if (i >= 2308)
    {
        std::ofstream outputFile("testNav.csv");
        std::ofstream outputFile2("testControllerState.csv");
        std::ofstream outputFile3("testTVC.csv");

        
        outputFile << "Time,";
        outputFile << "x,";
        outputFile << "y,";
        outputFile << "z,";
        outputFile << "vx,";
        outputFile << "vy,";
        outputFile << "vz,";
        outputFile << "phi,";
        outputFile << "theta,";
        outputFile << "psi,";
        outputFile << "p,";
        outputFile << "q,";
        outputFile << "r";
        outputFile << "\n";

        outputFile2 << "Time,";
        outputFile2 << "vx,";
        outputFile2 << "vy,";
        outputFile2 << "Iphi,";
        outputFile2 << "Itheta,";
        outputFile2 << "phi,";
        outputFile2 << "theta,";
        outputFile2 << "p,";
        outputFile2 << "q";
        outputFile2 << "\n";

        outputFile3 << "Time,";
        outputFile3 << "Input1,";
        outputFile3 << "Input2";
        outputFile3 << "\n";


        for (int i=0; i< 2308; i++)
        {
            for(int j = 0; j <= 12; j++)
            {
                if (j == 12)
                {
                    outputFile << testStates[i][j];
                }
                else
                {
                    outputFile << testStates[i][j] << ",";
                }
            }
            outputFile << "\n";
            
            for(int k = 0; k <= 8; k++)
            {
                if (k == 8)
                {
                    outputFile2 << testControllerStates[i][k];
                }
                else
                {
                    outputFile2 << testControllerStates[i][k] << ",";
                }
            }        
            outputFile2 << "\n";

            for(int l = 0; l <= 2; l++)
            {
                if (l == 2)
                {
                    outputFile3 << testInputs[i][l];
                }
                else
                {
                    outputFile3 << testInputs[i][l] << ",";
                }
            }
            outputFile3 << "\n";

        }
   
    outputFile.close();
    outputFile2.close();
    outputFile3.close();

    return Mode::Terminate;
    }


    

    
    navigation.UpdateNavigation(i);
    controller.UpdateLaunch(navigation, curr_time);
    
    testState = navigation.GetNavigation();
    testControllerState = controller.getXControl();
    testInput = controller.getInput();
    // std::cout<<testState<<"\n";

    
    testStateVector.push_back(curr_time);
    testControllerStateVector.push_back(curr_time);
    testInputsVector.push_back(curr_time);
        
    for(int j = 1; j <= 12; j++)
    {
        testStateVector.push_back(testState(j-1));
    } 
    testStates.push_back(testStateVector);
        
    for(int k = 1; k <=8; k++)
    {
        testControllerStateVector.push_back(testControllerState(k-1));
    }
    testControllerStates.push_back(testControllerStateVector);
        
    for(int l = 1; l <= 2; l++)
    {
        testInputsVector.push_back(testInput(l-1));
    }
    testInputs.push_back(testInputsVector);

   // std::tuple<double,double,double> acceleration = navigation.GetTestAcceleration(i);
    //std::cout<<std::get<0>(acceleration)<<"\n";
    //double acceleration_vector = (sqrt(pow(std::get<0>(acceleration),2) + pow(std::get<1>(acceleration), 2) + pow(std::get<2>(acceleration), 2)));
   // std::cout<<int(abs(acceleration_vector))<<"\n";
    if(testState(5) < 0 && testState(2) > 0.28){ 
        std::cout<<"We are switching to freefall"<<"\n";
        Telemetry::GetInstance().Log("Switching mode from launch to freefall");
        return Mode::Freefall;
    }
    else{
        return Mode::Launch;
    }

    // Added chang_time so updateLaunch includes iteratng through K
    
}

 Mode::Phase Mode::UpdateFreefall(Navigation& navigation, Igniter& igniter, double currTime, int i) {
     
    navigation.UpdateNavigation(i);
    Eigen::Matrix<double, 12, 1> currentState = navigation.GetNavigation();

    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;
    
    std::cout<<currTime <<"\n";

    if(currTime > MissionConstants::kFswCalibrationTime + motor_thrust_duration + offset){
       double a = -9.81/2;
       double b = currentState(5) + (-9.81*(motor_thrust_duration+motor_thrust_percentage));

       double average_landing_throttle = .08;

       double c = currentState(5) * (motor_thrust_duration * motor_thrust_percentage) + currentState(2) + -9.81*0.5*pow((motor_thrust_duration*motor_thrust_percentage),2) + average_landing_throttle*second_motor_delta_x - gse_height;
       std::cout<<c << "\n";
       std::cout<<currentState(5) << "\n";
       std::cout<<currentState(2) <<"\n";

       result = (-b - sqrt(pow(b,2) - 4*a*c)) / (2*a);

       time_till_second_ignite = result + offset;
       std::cout<<"Time till second ignite Command: " << time_till_second_ignite<<"\n";
    }
    else
    {
        return Mode::Freefall;
    }


     if (currTime > MissionConstants::kFswCalibrationTime + motor_thrust_duration + time_till_second_ignite){
        igniter.Ignite(Igniter::IgnitionSpecifier::LAND);
        return Mode::Terminate;

     }
 
    return Mode::Freefall;

     
 }

 Mode::Phase Mode::UpdateLand(Navigation& navigation, Controller& controller, double change_time){
//     static auto start_time = std::chrono::high_resolution_clock::now();
//     int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
//     double seconds = milliseconds_since_start / 1000.0;

//     navigation.UpdateNavigation(i);
//     controller.UpdateLaunch(navigation, seconds);

 //    std::tuple<double,double,double> acceleration = navigation.GetTestAcceleration(i);
 //    double acceleration_vector = (sqrt(pow(std::get<0>(acceleration),2) + pow(std::get<1>(acceleration), 2) + pow(std::get<2>(acceleration), 2)));
//     if ( 9.7 < acceleration_vector && acceleration_vector < 9.9 && 0.0 < navigation.GetHeight() && navigation.GetHeight() < 1.0)
//        return Mode::Terminate;

     return Mode::Land;
 }


bool Mode::Update(Navigation& navigation, Controller& controller, Igniter& igniter) {
    // Used for testing dead reckoning
    static int i = 0;
    // Used to record time for testing and for controller
    static double currTime = 0;
    //static auto startingTime = std::chrono::high_resolution_clock::now();
  
    static bool reset = false;
    static bool liftoff = false;

    
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto time_now = std::chrono::high_resolution_clock::now();
    double change_time = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - last_time).count() / 1000.0;
    //change_time = std::round(change_time * 1000.0) / 1000.0;
    last_time = time_now;
    while(change_time < 0.005)
    {
             change_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_time).count() / 1000.0;
            // change_time = std::round(change_time * 1000.0) / 1000.0;

    }
    if(change_time != 0.005)
    {
        change_time = 0.005;
    }
    currTime += change_time;


    /* Finish calculating time change*/

    //std::cout<<change_time<<"\n";
    navigation.loopTime = 0.005;
    controller.loopTime = 0.005;

   // std::cout << std::to_string(eCurrentMode) << "\n";
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
            this->eCurrentMode = UpdateIdle(navigation, controller,igniter, currTime, i, reset);
            i++;
            break;
        case Launch:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateLaunch(navigation, controller, currTime, i);
            i++;
            break;
        case Freefall:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateFreefall(navigation, igniter, currTime, i);
            i++;
            break;
        case Land:
            Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
            this->eCurrentMode = UpdateLand(navigation, controller, change_time);
            break;
        case Terminate:
            return false;
    }

    return true; 

}