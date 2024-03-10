// #include <chrono>
// #include <cmath>
// #include "Mode.hpp"
// #include "Navigation.hpp"
// #include "Telemetry.hpp"
// #include <iostream>


// //CONSTANTS TO BE FIGURED OUT LATER
// int abort_threshold = 1;
// int calibration_time = 1;
// int thrust_duration = 1;
// int descent_time = 1;
// int total_time = 1;
// double ignition_height = 1;
// double fsw = 0.005;

// Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

// Mode::Phase Mode::UpdateCalibration(Navigation& navigation, Controller& controller) {
//     static auto start_time = std::chrono::high_resolution_clock::now();
//     int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
//     double seconds = milliseconds_since_start / 1000.0;
//     static bool centered = false;

//     if(seconds >= 1){
//        // Telemetry::GetInstance().Log("Switching mode from calibration to idle");
//         navigation.importTestAccAndTestGyro();
//         controller.ImportControlParameters("../12-9-k-matrix.csv");
//         return Mode::Idle;

//     }
    
//     return Mode::Calibration;
// }

// Mode::Phase Mode::UpdateIdle(Navigation& navigation, Controller& controller) {

//     static auto start_time = std::chrono::high_resolution_clock::now();
//     int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
//     double seconds = milliseconds_since_start / 1000.0;

//     if(seconds > 1){
//         Telemetry::GetInstance().Log("Switching mode from idle to launch");
//         static auto start_time = std::chrono::high_resolution_clock::now();

//         return Mode::Launch;
//     }


//     return Mode::Idle;
// }

// Mode::Phase Mode::UpdateLaunch(Navigation& navigation, Controller& controller, double curr_time, int i) {
//     static Eigen::Matrix<double, 601, 13> testStates;
//      Eigen::Matrix<double, 12, 1> testState;

//      double time = curr_time;

//    // Create a static variable to initalize the Start time so cotroller can call start the first time this method is called   

//     if (i == 601)
//     {
//         char separator = ',';
//         std::string row, row2, item, item2;
//         std::ofstream outputFile("testNav.csv");

//         outputFile << "Time,";
//         outputFile << "x,";
//         outputFile << "y,";
//         outputFile << "z,";
//         outputFile << "vx,";
//         outputFile << "vy,";
//         outputFile << "vz,";
//         outputFile << "phi,";
//         outputFile << "theta,";
//         outputFile << "psi,";
//         outputFile << "p,";
//         outputFile << "q,";
//         outputFile << "r";
//         outputFile << "\n";


//         // Start at index 1 to allow row for names
//         for (int i=0; i< 601; i++)
//         {
//             for(int j = 0; j <= 12; j++)
//             {
//                 if (j == 12)
//                 {
//                     outputFile << testStates(i,j);
//                 }
//                 else
//                 {
//                     outputFile << testStates(i,j) << ",";
//                 }
//             }
//             outputFile << "\n";
//         }
   
//     outputFile.close();
//     return Mode::Terminate;
//     }

//     static int iteration = 1;
//     if(iteration > 0){
//         controller.Start(curr_time);
//         iteration--;
//     }
//     navigation.UpdateNavigation(i);
//    //controller.UpdateLaunch(navigation, curr_time);

//     testState = navigation.GetNavigation();
//     // std::cout<<testState<<"\n";
//      testStates(i,0) = curr_time;
//      for(int j = 1; j <= 12; j++)
//      {
//         testStates(i,j) = testState(j-1);
//      } 

//     return Mode::Launch;

//     // Added chang_time so updateLaunch includes iteratng through K
    

// }



// bool Mode::Update(Navigation& navigation, Controller& controller) {
//     // Used for testing dead reckoning
//     static int i = 0;
//     // Used to record time for testing and for controller
//     static double currTime = 0;
//     //static auto startingTime = std::chrono::high_resolution_clock::now();
    
//     static auto last_time = std::chrono::high_resolution_clock::now();
//     auto time_now = std::chrono::high_resolution_clock::now();
//     double change_time = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - last_time).count() / 1000.0;
//     last_time = time_now;
//     while(change_time < 0.005)
//     {
//              change_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_time).count() / 1000.0;

//     }
//     currTime += change_time;
//     /* Finish calculating time change*/

//     //std::cout<<change_time<<"\n";
//     navigation.loopTime = change_time;
//     controller.loopTime = change_time;

//    // std::cout << std::to_string(eCurrentMode) << "\n";


//     /* Handle behavior based on current phase. Update phase*/
//     switch(this->eCurrentMode)
//     {
//         case Calibration:
//             Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05);
//             this->eCurrentMode = UpdateCalibration(navigation, controller);
//             break;
//         case Idle:
//             Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.05);
//             this->eCurrentMode = UpdateIdle(navigation, controller);
//             break;
//         case Launch:
//             Telemetry::GetInstance().RunTelemetry(navigation, controller, 0.01);
//             this->eCurrentMode = UpdateLaunch(navigation, controller,currTime, i);
//             i++;
//             break;
//         case Terminate:
//             return false;
//     }

//     return true; 

// }