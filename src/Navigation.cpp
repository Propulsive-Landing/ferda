#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <vector>
#include <iostream>
#include <chrono>
#include <iomanip>


#include "Navigation.hpp"
#include "MissionConstants.hpp"

Navigation::Navigation(IMU& imu, Barometer& barometer, TVC& tvc) : imu(imu), barometer(barometer), tvc(tvc) 
{
    std::cout << std::setprecision(4) << std::fixed;
    stateMat = Eigen::Matrix<double, 12, 1>::Zero();
    // Set z position to rocket com
    stateMat(2) = 0.28;
    pressureInit = barometer.GetPressure();
}

void Navigation::reset()
{
    stateMat = Eigen::Matrix<double, 12, 1>::Zero();
    stateMat(2) = 0.28;
    d_theta_queue_reckon.clear();
}

Eigen::Matrix<double, 12, 1> Navigation::GetNavigation()
{
    return stateMat;
}

double Navigation::GetHeight() {
    double pressure = barometer.GetPressure();
    double temp = barometer.GetTemperature(); 
    temp += 273.15; // Convert to K;
    return (log(pressure/pressureInit) * 8.3145 * temp) / (0.02897 * -9.81);
    // Pressure is in kpa
}
    
void Navigation::UpdateNavigation(){
    // Updates stateMat //

    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;
   
    // Create 2 tuples to hold the the linear acceleration and angular rate data from the imu 
    std::tuple<double,double,double> linearAcceleration = imu.GetBodyAcceleration();
    std::tuple<double,double,double> angularRate = imu.GetBodyAngularRate();
   
    // Convert the linear acceleration tuple to a Vector so we can muliply the Eigen matrix R by another Eigen type which in this case is a vector
    Eigen::Vector3d linearAccelerationVector(std::get<0>(linearAcceleration), std::get<1>(linearAcceleration), std::get<2>(linearAcceleration));
    //std::cout<< linearAccelerationVector "\n";;
    // Get phi, theta, and psi
    double phi = stateMat(6);
    double theta = stateMat(7);
    double psi = stateMat(8);

    // Convert the three euler angles to a rotation matrix that can move a vector from the body fixed frame into the ground fixed frame
    Eigen::Matrix<double, 3, 3> R = CreateRotationalMatrix(phi, theta, psi);

    // Update the linear positions
    stateMat.segment(0,3) += stateMat.segment(3,3) * loopTime;
    
    // Update the linear velocities
    stateMat.segment(3,3) += R * linearAccelerationVector * loopTime;

   // newState(5) = newState(5) - 9.81*loopTime;
    stateMat(5) -=  9.81* loopTime;

    // Update the angles
    stateMat.segment(6,3) += stateMat.segment(9,3) * loopTime;

    // Create a vector that will hold d_theta and set all of the elements to 0 and get the angular rate
    std::vector<double> d_theta_now = {0,0,0};
    d_theta_now[0] = std::get<0>(angularRate) + std::get<1>(angularRate)*sin(phi)*tan(theta)+ std::get<2>(angularRate)*cos(phi)*tan(theta);
    d_theta_now[1] =  std::get<1>(angularRate)*cos(phi) - std::get<2>(angularRate)*sin(phi);
    d_theta_now[2] =  std::get<1>(angularRate)*sin(phi)* (1/(cos(theta))) + std::get<2>(angularRate)*cos(phi)* (1/(cos(theta)));
    //std::cout<< d_theta_now[0]  << "\n";
    // Append the vector, d_theta_now, to d_theta_queue_reckon
    d_theta_queue_reckon.push_back(d_theta_now);
    
    
    // Call ComputeAngularRollingAverage to sum up all of the data so far for p,q,r which represent the angular velocity in x, y, and z direction
    std::tuple<double,double,double> rollingAngularAverage = ComputeAngularRollingAverage();
   // std::cout<< std::get<0>(rollingAngularAverage) << ", " << std::get<1>(rollingAngularAverage) << ", " << std::get<2>(rollingAngularAverage)<<"\n";
    //Assign the sum to their respective states, that being p,q, and r
    stateMat(9) = std::get<0>(rollingAngularAverage);
    stateMat(10) = std::get<1>(rollingAngularAverage);
    stateMat(11) = std::get<2>(rollingAngularAverage);

   
}

std::tuple<double,double,double> Navigation::ComputeAngularRollingAverage(){
    // Computes a rolling average of the angular velocities //
    
    // Calculate the maximum amount of entries that d_theta_queue_reckon can have
    int max_theta_dot_smooth_entries = MissionConstants::kNavThetaDotSmooth/loopTime;

    // Determine if the amount of entries in d_theta_reckon is greater than max_theta_dot_smooth_entries,
    // and if that is true, pop the first entry 
    if(d_theta_queue_reckon.size() > max_theta_dot_smooth_entries){
        d_theta_queue_reckon.pop_front();
    }

    // Sum up all of the data so far for p,q,r which represent the angular velocity in x, y, and z direction
    double p = 0, q = 0, r = 0;
   
    for (int i = 0; i < d_theta_queue_reckon.size(); i++)
    {
        p += d_theta_queue_reckon[i][0] / d_theta_queue_reckon.size();
        q += d_theta_queue_reckon[i][1] / d_theta_queue_reckon.size();
        r += d_theta_queue_reckon[i][2] / d_theta_queue_reckon.size();
    }


    // Return a tuple of the rolling average of p,q, and r
    return std::make_tuple(p,q,r);
}

Eigen::Matrix3d Navigation::CreateRotationalMatrix(double phi, double theta, double psi){
    // Update the roational matrix that is used to transform the body frame to the ground frame //

    Eigen::Matrix3d rotationalMatrix;

    rotationalMatrix(0,0) = cos(theta)*cos(psi);
    rotationalMatrix(0,1) = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    rotationalMatrix(0,2) = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    rotationalMatrix(1,0) = cos(theta)*sin(psi);
    rotationalMatrix(1,1) = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    rotationalMatrix(1,2) = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    rotationalMatrix(2,0) = -sin(theta);
    rotationalMatrix(2,1) = sin(phi)*cos(theta);
    rotationalMatrix(2,2) = cos(phi)*cos(theta);

    return rotationalMatrix;

}

std::tuple<double, double, double> Navigation::GetBodyAcceleration()
{
    return imu.GetBodyAcceleration();
}

