#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <vector>
#include <iostream>

 
#include "Navigation.hpp"

//CONSTANTS TO BE FIGURED OUT LATER
double nav_theta_dot_smooth = 1;
double fsw_loop_time = 1;


Navigation::Navigation(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter) : imu(imu), barometer(barometer), tvc(tvc), igniter(igniter), count(0) 
{
    
}

Eigen::Matrix<double, 12, 1> Navigation::GetNavigation()
{
    return stateMat;
}

    
void Navigation::UpdateNavigation(){
    // Updates stateMat //
   
   
    // Create a matrix to hold the new State value and then set all of the elements to 0
    Eigen::Matrix<double, 12, 1> newState;
    newState.setZero();

    // Create 2 tuples to hold the the linear acceleration and angular rate data from the imu 
    std::tuple<double,double,double> linearAcceleration = imu.GetBodyAcceleration();
    std::tuple<double,double,double> angularRate = imu.GetBodyAngularRate();

    // Convert the linear acceleration tuple to a Vector so we can muliply the Eigen matrix R by another Eigen type which in this case is a vector
    Eigen::Vector3d linearAccelerationVector(std::get<0>(linearAcceleration), std::get<1>(linearAcceleration), std::get<2>(linearAcceleration));
   
    // Get phi, theta, and psi
    double phi = stateMat(6);
    double theta = stateMat(7);
    double psi = stateMat(8);

    // Convert the three euler angles to a rotation matrix that can move a vector from the body fixed frame into the ground fixed frame
    Eigen::Matrix<double, 3, 3> R = CreateRotationalMatrix(phi, theta, psi);

    // Update the linear positions
    newState.segment(0,3) = stateMat.segment(3,3) * fsw_loop_time + stateMat.segment(0,3);
    // Update the linear velocities
    newState.segment(3,3) = R * linearAccelerationVector * fsw_loop_time + stateMat.segment(3,3);
    // Account for gravity
    newState(5) = newState(5) - 9.81*fsw_loop_time;
    // Update the angles
    newState.segment(6,3) = stateMat.segment(9,3) * fsw_loop_time + stateMat.segment(6,3);

    // Create a vector that will hold d_theta and set all of the elements to 0 and get the angular rate
    std::vector<double> d_theta_now = {0,0,0};
    d_theta_now[0] = std::get<0>(angularRate) + std::get<1>(angularRate)*sin(phi)*tan(theta)+ std::get<2>(angularRate)*cos(phi)+tan(theta);
    d_theta_now[1] =  std::get<1>(angularRate)*cos(phi) - std::get<2>(angularRate)*sin(phi);
    d_theta_now[2] =  std::get<1>(angularRate)*sin(phi)* (1/(cos(theta))) + std::get<2>(angularRate)*cos(phi)* (1/(cos(theta)));
    
    // Append the vector, d_theta_now, to d_theta_queue_reckon
    d_theta_queue_reckon.push_back(d_theta_now);

    // Update count to represent what entry d_theta_queue_reckon is on
    count += 1;
    
    // Call ComputeAngularRollingAverage to sum up all of the data so far for p,q,r which represent the angular velocity in x, y, and z direction
    std::tuple<double,double,double> rollingAngularAverage = ComputeAngularRollingAverage();
    
    //Assign the sum to their respective states, that being p,q, and r
    newState(9) = std::get<0>(rollingAngularAverage);
    newState(10) = std::get<1>(rollingAngularAverage);
    newState(11) = std::get<2>(rollingAngularAverage);
   
    // Assign the new state to stateMat;
    stateMat = newState;
}

void Navigation::Reset(){
    // Resets count, the state, and d_theta_queue_reckon //

    count = 0;
    stateMat.setZero();
    d_theta_queue_reckon.clear();
}

std::tuple<double,double,double> Navigation::ComputeAngularRollingAverage(){
    // Computes a rolling average of the angular velocities //
    
    // Calculate the maximum amount of entries that d_theta_queue_reckon can have
    int max_theta_dot_smooth_entries = nav_theta_dot_smooth/fsw_loop_time;

    // Determine if the amount of entries in d_theta_reckon is less than or equal to max_theta_dot_smooth_entries, and if that is true, 
    // set divisor to the amount of entries in d_theta_reckon. If it's not true, pop the first entry and set divisor
    // to the maximum number of entries which is max_theta_dot_smooth_entries
    int divisor = 0;
    if (count <= max_theta_dot_smooth_entries)
    {
         divisor = count;
    }
    else 
    {
        d_theta_queue_reckon.pop_front();
        divisor = max_theta_dot_smooth_entries;
    }

    // Sum up all of the data so far for p,q,r which represent the angular velocity in x, y, and z direction
    double p = 0, q = 0, r = 0;
   
    for (int i = 0; i < divisor; i++)
    {
        p += d_theta_queue_reckon[0][i];
        q += d_theta_queue_reckon[1][i];
        r += d_theta_queue_reckon[2][i];
    }

    // Get an average by dividing over the number of entries so far
    p /= divisor;
    q /= divisor;
    r /= divisor;

    // Return a tuple of the rolling average of p,q, and r
    return std::make_tuple(p,q,r);
}

Eigen::Matrix3d Navigation::CreateRotationalMatrix(double phi, double theta, double psi){
    // Update the roational matrix that is used to transform the body frame to the ground frame //

    Eigen::Matrix3d rotationalMatrix;

    rotationalMatrix(0,0) = cos(theta)*cos(psi);
    rotationalMatrix(0,1) = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    rotationalMatrix(0,2) = cos(psi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    rotationalMatrix(1,0) = cos(theta)*sin(psi);
    rotationalMatrix(1,1) = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    rotationalMatrix(1,2) = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    rotationalMatrix(2,0) = -sin(theta);
    rotationalMatrix(2,1) = sin(phi)*cos(theta);
    rotationalMatrix(2,2) = cos(phi)*cos(theta);

    return rotationalMatrix;

}