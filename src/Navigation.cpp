#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <vector>
#include <iostream>


#include "Navigation.hpp"

Navigation::Navigation(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter) : imu(imu), barometer(barometer), tvc(tvc), igniter(igniter) 
{
}

Eigen::Matrix<double, 12, 1> Navigation::GetNavigation()
{
    return stateMat;
}

void Navigation::UpdateNavigation(){
    // Create a matrix to hold the new State value and then set all of the elements to 0
    Eigen::Matrix<double, 12, 1> newState;
    newState.setZero();

    // Create 2 tuples to hold the the linear acceleration and angular rate data from the imu 
    std::tuple<double,double,double> linearAcceleration = imu.GetBodyAcceleration();
    std::tuple<double,double,double> angularRate = imu.GetBodyAngularRate();

    // Convert those tuples to arrays which will be used later for calculations
    double linearAccelerationArray[] = {std::get<0>(linearAcceleration), std::get<1>(linearAcceleration), std::get<2>(linearAcceleration)};
    double angularRateArray[] = {std::get<0>(angularRate), std::get<1>(angularRate), std::get<2>(angularRate)};

    // Convert the linear acceleration array to a Matrix so we can muliply the Eigen matrix R by another Eigen Matrix with the same dimensions
    Eigen::Matrix<double,3,3> linearAccelerationMatrix;
    linearAccelerationMatrix << linearAccelerationArray[0], 0 ,0,
                                linearAccelerationArray[1], 0, 0,
                                linearAccelerationArray[2], 0, 0;
    
    //CONSTANTS TO BE FIGURED OUT LATER
    double nav_theta_dot_smooth = 1;
    double fsw_loop_time = 1;

    // Calculate n
    int n = nav_theta_dot_smooth/fsw_loop_time;
   
    // Create a static vector that has deque properties (meaning you can add and delete from the front and back) 
    // to hold d_theta_queue_reckon and make sure it retains its values throughout function calls
    static std::deque<std::vector<double>> d_theta_queue_reckon;

    // Create a static variable count for the calculations for smoothing angular velocity and update it every time this function is called
    // to represent what entry d_theta_now is on
    static int count = 0;
    count += 1;

    // Get phi, theta, and psi
    double phi = stateMat(6);
    double theta = stateMat(7);
    double psi = stateMat(8);

    // Convert the three euler angles to a rotation matrix that can move a vector from the body fixed frame into the ground fixed frame
    Eigen::Matrix<double, 3, 3> R;
    R(0,0) = cos(theta)*cos(psi);
    R(0,1) = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    R(0,2) = cos(psi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    R(1,0) = cos(theta)*sin(psi);
    R(1,1) = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    R(1,2) = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    R(2,0) = -sin(theta);
    R(2,1) = sin(phi)*cos(theta);
    R(2,2) = cos(phi)*cos(theta);

    // Update the linear positions
    newState.segment(0,3) = stateMat.segment(3,3) * fsw_loop_time + stateMat.segment(0,3);
    // Update the linear velocities
    Eigen::Vector3d vectorProduct = linearAccelerationMatrix * R * Eigen::Vector3d::Ones();
    newState.segment(3,3) = vectorProduct * fsw_loop_time + stateMat.segment(3,3);
    // Account for gravity
    newState(5) = newState(5) - 9.81*fsw_loop_time;
    // Update the angles
    newState.segment(6,3) = stateMat.segment(9,3) * fsw_loop_time + stateMat.segment(6,3);

    // Create a vector that will hold d_theta and set all of the elements to 0
    std::vector<double> d_theta_now = {0,0,0};
    // Get the angular rates
    d_theta_now[0] = angularRateArray[0] + angularRateArray[1]*sin(phi)*tan(theta)+angularRateArray[2]*cos(phi)+tan(theta);
    d_theta_now[1] = angularRateArray[1]*cos(phi) - angularRateArray[2]*sin(phi);
    d_theta_now[2] = angularRateArray[1]*sin(phi)* (1/(cos(theta))) + angularRateArray[2]*cos(phi)* (1/(cos(theta)));
    
    // Add the vector, d_theta_now, to d_theta_queue_reckon
    d_theta_queue_reckon.push_back(d_theta_now);

    // Determine if the amount of entries in d_theta_reckon is less than or equal to n, and if that is true, 
    // set divisor to the amount of entries in d_theta_reckon. If it's not true, pop the first entry and set divisor
    // to the maximum number of entries which is n
    int divisor = 0;
    if (count <= n)
    {
         divisor = count;
    }
    else 
    {
        d_theta_queue_reckon.pop_front();
        divisor = n;
    }

    // Sum up all of the data so far for p,q,r which represent the angular velocity in x, y, and z direction
    double p = 0;
    double q = 0;
    double r = 0;
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

    //Assign the angular velocity to their respective state
    newState(9) = p;
    newState(10) = q;
    newState(11) = r;
   
    // Assign the new state to stateMat;
    stateMat = newState;
    
}
