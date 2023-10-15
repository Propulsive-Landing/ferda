#include <Eigen/Dense>

#include "Navigation.hpp"

Navigation::Navigation(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter) : imu(imu), barometer(barometer), tvc(tvc), igniter(igniter) 
{
}

Eigen::Matrix<double, 12, 1> Navigation::GetNavigation()
{
    // TODO Return the current navigation state

    Eigen::Matrix<double, 12, 1> m = {      // construct a 2x2 matrix
      {1},     // first row
      {3},      // second row
      {5},      // Third row
    };
   
   return m;

}

void Navigation::UpdateNavigation(){
    // TODO Calculate navigation state (angle, position, etc) from the input 
    // TODO Update the navigation state 
}
