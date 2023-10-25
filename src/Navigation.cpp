#include <Eigen/Dense>
#include <cmath>

#include "Navigation.hpp"

Navigation::Navigation(IMU& imu, Barometer& barometer, TVC& tvc, Igniter& igniter) : imu(imu), barometer(barometer), tvc(tvc), igniter(igniter) 
{
}

Eigen::Matrix<double, 12, 1> Navigation::GetNavigation()
{
    return stateMat;
}

void Navigation::UpdateNavigation(){
    // Create a matrix to hold the new State value
    Eigen::Matrix<double, 12, 1> newState;

    // Create 2 tuples to hold the the linear acceleration and angular rate data from the imu 
    std::tuple<double,double,double> linearAcceleration = imu.GetBodyAcceleration();
    std::tuple<double,double,double> angularRate = imu.GetBodyAngularRate();

    // Convert those tuples to arrays which will be used later for calculations
    double linearAccelerationArray[] = {std::get<0>(linearAcceleration), std::get<1>(linearAcceleration), std::get<2>(linearAcceleration)};
    double angularRateArray[] = {std::get<0>(angularRate), std::get<1>(angularRate), std::get<2>(angularRate)};

    // Set all of the elemnts in newState to 0
    newState.setZero();

    //CONSTANTS TO BE FIGURED OUT LATER
    double nav_theta_dot_smooth;
    double fsw_loop_time;

    // Calculate n
    int n = nav_theta_dot_smooth/fsw_loop_time;
   
    // Create a Matrix to hold d_theta_queue and the count and make these static so they hold their status when we call this function
    static Eigen::MatrixXd d_theta_queue_reckon(3,n);
    static int count = 0;
    d_theta_queue_reckon.setZero();


    // Get phi, theta, and psi
    int phi = stateMat(6);
    int theta = stateMat(7);
    int psi = stateMat(8);

    // Create a Matrix R that will be the conversion from the navigation frame to the body frame
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

    // Convert the linear acceleration array to a vector
    Eigen::Vector3d linearAccelerationVector = Eigen::Map<Eigen::Vector3d>(linearAccelerationArray); 
   
    // Update the linear velocities
    newState.segment(4,3) = linearAccelerationVector * R * fsw_loop_time + stateMat.segment(4,3);
    
    // Account for gravity
    newState(6) = newState(6) - 9.81*fsw_loop_time;

    // Update the angles
    newState.segment(6,3) = stateMat.segment(10,3) * fsw_loop_time + stateMat.segment(6,3);

    // Create a Matrix that will hold d_theta
    Eigen::Matrix<double, 3, 1> d_theta_now;
    d_theta_now.setZero();

    // Get the angular rates
    d_theta_now(1) = angularRateArray[0] + angularRateArray[1]*sin(phi)*tan(theta)+angularRateArray[2]*cos(phi)+tan(theta);
    d_theta_now(2) = angularRateArray[1]*cos(phi) - angularRateArray[2]*sin(phi);
    d_theta_now(3) = angularRateArray[1]*sin(phi)* (1/(cos(theta))) + angularRateArray[2]*cos(phi)* (1/(cos(theta)));
    
    // Resize d_theta_queue_reckon if needed and sum up all of the elements starting from the column 2
    d_theta_queue_reckon.resize(d_theta_queue_reckon.rows() * d_theta_queue_reckon.cols(), d_theta_now.rows() * d_theta_now.cols());
    d_theta_queue_reckon = d_theta_queue_reckon.block(0,1, d_theta_queue_reckon.rows(), n + 1);

    // TODO
    int divisor;
    if (count + 1 < n)
    {
         divisor = count + 1;
    }
    else
    {
        divisor = n;
    }
    Eigen::VectorXd d_theta_smoothed = d_theta_queue_reckon.rowwise().sum();

    newState.segment(9,3) = d_theta_smoothed;

    // Assign the new state to stateMat;
    stateMat = newState;
    
}
