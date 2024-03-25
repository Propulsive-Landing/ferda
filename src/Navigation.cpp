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
   stateMat(2) = 0.28;
   pressureInit = 102911.127410746;



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

double Navigation::GetHeight(int i) {
    double pressure = GetTestBarom(i);
    double temp = 288;
    return (log(pressure/pressureInit) * 8.314462618 * temp) / (0.02896968 * -9.80298);
}
    
void Navigation::UpdateNavigation(int i){
    // Updates stateMat //
   

    // Create 2 tuples to hold the the linear acceleration and angular rate data from the imu 
   // std::tuple<double,double,double> linearAcceleration = imu.GetBodyAcceleration();
    //std::tuple<double,double,double> angularRate = imu.GetBodyAngularRate();
     std::tuple<double,double,double> linearAcceleration = GetTestAcceleration(i);
     std::tuple<double,double,double> angularRate = GetTestGyroscope(i);

    // Convert the linear acceleration tuple to a Vector so we can muliply the Eigen matrix R by another Eigen type which in this case is a vector
    Eigen::Vector3d linearAccelerationVector(std::get<0>(linearAcceleration), std::get<1>(linearAcceleration), std::get<2>(linearAcceleration));
    // Get phi, theta, and psi
    double phi = stateMat(6);
    //std::cout<<phi<<"\n";
    double theta = stateMat(7);
       // std::cout<<theta<<"\n";

    double psi = stateMat(8);
       // std::cout<<psi<<"\n";


    // Convert the three euler angles to a rotation matrix that can move a vector from the body fixed frame into the ground fixed frame
    Eigen::Matrix<double, 3, 3> R = CreateRotationalMatrix(phi, theta, psi);
    
    if (i == 602){
    //std::cout<< R<<"\n";
    }

    // Update the linear positions
    //newState.segment(0,3) = stateMat.segment(3,3) * loopTime + stateMat.segment(0,3);
    stateMat.segment(0,3) += stateMat.segment(3,3) * loopTime;
    
    // Update the linear velocities
    //newState.segment(3,3) = R * linearAccelerationVector * loopTime + stateMat.segment(3,3);
    stateMat.segment(3,3) += R * linearAccelerationVector * loopTime;

   // newState(5) = newState(5) - 9.81*loopTime;
    stateMat(5) -=  9.81* loopTime;

    // Update the angles
   // newState.segment(6,3) = stateMat.segment(9,3) * loopTime + stateMat.segment(6,3);
    stateMat.segment(6,3) += stateMat.segment(9,3) * loopTime;

   
    // Create a vector that will hold d_theta and set all of the elements to 0 and get the angular rate
    std::vector<double> d_theta_now = D_Theta_Now_Math(phi, theta, psi, angularRate);
   
    // Append the vector, d_theta_now, to d_theta_queue_reckon
    d_theta_queue_reckon.push_back(d_theta_now);
    
    
    
    // Call ComputeAngularRollingAverage to sum up all of the data so far for p,q,r which represent the angular velocity in x, y, and z direction
    std::tuple<double,double,double> rollingAngularAverage = ComputeAngularRollingAverage();
    
     if(i == 602) {   
     
       // std::cout<<"Smooth p = " << d_theta_queue_reckon[d_theta_queue_reckon.size() - 1][0] << "\n";
       // std::cout<<"Smooth q = " << d_theta_queue_reckon[d_theta_queue_reckon.size() - 1][1] << "\n";
        //std::cout<<"Smooth r = " << d_theta_queue_reckon[d_theta_queue_reckon.size() - 1][2] << "\n";
     }
     
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
   // std::cout<<double(cos(0.0))<<"\n";

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

std::vector<double> Navigation::D_Theta_Now_Math(double phi, double theta, double psi, std::tuple<double,double,double> angularRate){
    // Computes math to get velocity in roll, pitch, and yaw directions

    std::vector<double> d_theta_now = {0,0,0};
    //std::cout<< phi << "\n";
    //std::cout<< std::cos(phi) << "\n";
    //std::cout<< std::cos(phi) << "\n";
    //std::cout<<std::sin(phi)<<"\n";
    //std::cout<<std::sin(0.0001)<<"\n";
  //  bool t = phi == 0.0001;
   // std::cout<<t<<"\n";

    d_theta_now[0] = std::get<0>(angularRate) + std::get<1>(angularRate)*sin(phi)*tan(theta)+ std::get<2>(angularRate)*cos(phi)*tan(theta);
    d_theta_now[1] =  std::get<1>(angularRate)*cos(phi) - std::get<2>(angularRate)*sin(phi);
    d_theta_now[2] =  std::get<1>(angularRate)*sin(phi)* (1/(cos(theta))) + std::get<2>(angularRate)*cos(phi)* (1/(cos(theta)));
   
   // std::cout<<"d_theta_now[0]" << d_theta_now[0] << "\n";
   // std::cout<<"d_theta_now[1]" << d_theta_now[1] << "\n";
    //std::cout<<"d_theta_now[2]" << d_theta_now[2] << "\n";
    

    return d_theta_now;
}

std::tuple<double, double, double> Navigation::GetTestAcceleration(int i)
{
      std::tuple<double,double,double> linearAcceleration = {linearAccels[i][0], linearAccels[i][1], linearAccels[i][2]};
      return linearAcceleration;
}

std::tuple<double, double, double> Navigation::GetTestGyroscope(int i)
{
     std::tuple<double,double,double> angularAcceleration = {gyroAccels[i][0], gyroAccels[i][1], gyroAccels[i][2]};
      return angularAcceleration;
}

double Navigation::GetTestBarom(int i)
{
    return baromValues[i][0];
}

void Navigation::importTestAccAndTestGyro()
{
    char separator = ',';
    std::string row, row2, item, item2;
    std::ifstream in("../accelerometer.csv");
    std::ifstream in2("../gyroscope.csv");

    for (int i=0; i< 2308; i++){
        std::getline(in, row);
       // std::cout<<row<<"\n";
        std::getline(in2, row2);

        std::stringstream ss(row);
        std::stringstream ss2(row2);

        std::vector<double> getRow;
        std::vector<double> getRow2;

        
        for (int j=0; j<3; j++){    
            std::getline(ss, item, separator);
            std::getline(ss2, item2, separator);
            std::stringstream ss(row);
            getRow.push_back(stod(item));
            std::stringstream ss2(row2);
            getRow2.push_back(stod(item2));
        }
        linearAccels.push_back(getRow);
        gyroAccels.push_back(getRow2);
    }
}
void Navigation::importTestBarom()
{
        std::string row3 ,item3;
        std::ifstream in3("../barometer.csv");

        for (int i=0; i< 2308; i++)
        {
            std::getline(in3, row3);
            std::stringstream ss3(row3);
            std::vector<double> getRow3;

            std::getline(ss3, item3);
            getRow3.push_back(stod(item3));
            baromValues.push_back(getRow3);

}





}



/*
    0.0029
   -0.5204
   77.3056
    0.0027
   -0.3528
   26.7202
   -0.0001
   -0.0004
    0.0021
    0.0001
   -0.0010
    0.0004    
 R: 
    1.0000   -0.0021   -0.0004
    0.00211.00000.0001
    0.0004   -0.00011.0000
d_theta_now: 
    0.0001
   -0.0051
    0.0010
Output:    
    0.0029
   -0.5222
   77.4392
    0.0028
   -0.3533
   26.6690
   -0.0001
   -0.0004
    0.0021
    0.0002
   -0.0014
    0.0003

    0.00181714,
    -0.546316,
    76.8751,
    0.00212192,
    -0.361886,
    26.67,
    -4.64491e-05,
    -0.000443881,
    0.00213429,
    0.000116037,
    -0.000969894,
    0.000370655

*/

