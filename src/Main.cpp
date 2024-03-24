
#include "Barometer.hpp"
#include "IMU.hpp"
#include "TVC.hpp"

#include "Navigation.hpp"
#include "Controller.hpp"

#include "Telemetry.hpp"
#include "Igniter.hpp"

#include "Mode.hpp"
#include "MissionConstants.hpp"

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <vector>
#include <iomanip>



#ifdef NDEBUG
    #include <pigpio.h>
#endif

int main()
{
       std::cout << std::setprecision(4) << std::fixed;

    #ifdef NDEBUG
        if (gpioInitialise() < 0)
            throw std::runtime_error("failed to initialize gpio");

        gpioSetMode(18, PI_OUTPUT);
        gpioSetMode(13, PI_OUTPUT);
    #endif

    IMU imu;
    Barometer barometer;
    TVC tvc;
    Igniter igniter;



    Controller controller(tvc);
//      Eigen::Matrix<double, 12, 1> testState;
     
//      Eigen::Matrix<double, 602, 13> testStates;

       Navigation navigation(imu, barometer, tvc);
//          testState = navigation.GetNavigation();
//             //   std::cout<<testState(6)<<"\n";
//             //   std::cout<<testState(7)<<"\n";
//             //   std::cout<<testState(8)<<"\n";

//             //  std::cout<<"\n";

//             //   std::cout<<testState(9)<<"\n";
//             //   std::cout<<testState(10)<<"\n";
//             //   std::cout<<testState(11)<<"\n";
//             //  std::cout<<"\n";


//     navigation.importTestAccAndTestGyro();
//      double lastTime = 0;
//     double time = 0; 
//     double dt = 0;
//     static auto start_time = std::chrono::high_resolution_clock::now();
//      double milliseconds_since_start = 0;
//      double next_time = 0;
   
//     for(int i =0; i < 602; i++)
//    {
//      lastTime = milliseconds_since_start;
//      milliseconds_since_start = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count()) / 1000.0;
//      dt = milliseconds_since_start - lastTime;
//      next_time = milliseconds_since_start + 0.005;
//      navigation.UpdateNavigation(i);
//      testState = navigation.GetNavigation();
//      if (i==601){
//      std::cout<<testState<<"\n";
//      }

//      while((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count()) / 1000.0 < next_time)
//      {
//          continue;
//      }
//      //std::cout<<dt<<"\n";
//      testStates(i,0) = milliseconds_since_start;

//        for(int j = 1; j <= 12; j++)
//        {
//           testStates(i,j) = testState(j-1);
//        }

//     }
   
 
    
//     char separator = ',';
//     std::string row, row2, item, item2;
//     std::ofstream outputFile("testNav.csv");

//      outputFile << "Time,";
//      outputFile << "x,";
//      outputFile << "y,";
//      outputFile << "z,";
//      outputFile << "vx,";
//      outputFile << "vy,";
//      outputFile << "vz,";
//      outputFile << "phi,";
//      outputFile << "theta,";
//      outputFile << "psi,";
//      outputFile << "p,";
//      outputFile << "q,";
//      outputFile << "r";
//      outputFile << "\n";


//     // Start at index 1 to allow row for names
//     for (int i=0; i< 602; i++)
//     {
//         for(int j = 0; j <= 12; j++)
//         {
//             if (j == 12)
//             {
//                 outputFile << testStates(i,j);
//             }
//             else
//             {
//                 outputFile << testStates(i,j) << ",";
//             }
//         }
//         outputFile << "\n";
//     }
//     outputFile.close();

    
        
        
    //Telemetry::GetInstance().Log("Starting program...");

    // TODO we need to set controller iteration gains or there is a segmentation fault.



    Mode mode(Mode::Calibration);
    
    //while(mode.Update(navigation, controller, igniter)) {}
    navigation.importTestBarom();
    for(int i = 0; i < 2308; i++)
    {
        std::cout<< navigation.GetHeight(i)<<"\n";
    }

    //#ifdef NDEBUG
    //    gpioTerminate();
   // #endif
   
    return 0;
}
