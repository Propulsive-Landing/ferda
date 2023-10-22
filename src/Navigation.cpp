#include <Eigen/Dense>

#include "Navigation.hpp"

Navigation::Navigation(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter) : imu(imu), barometer(barometer), tvc(tvc), igniter(igniter) 
{
}

Eigen::Matrix<double, 12, 1> Navigation::GetNavigation()
{
    return stateMat;
}

void Navigation::UpdateNavigation(){
    Eigen::Matrix<double, 12, 1> newState;
    // TODO Calculate navigation state (angle, position, etc) from the input 
    // TODO Update the navigation state
    stateMat = newState;
}
