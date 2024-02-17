#include <iostream>
#include <cmath>
#include <array>
#include <memory>
#include <thread>
#include "IMU.hpp"
#include <chrono>


int main() {
    IMU imu;
    bool Exitloop = false;
	std::thread imuThread{[&imu, &Exitloop]()
	{
        while (!Exitloop)
		{
			std::tuple<double,double,double> angularRate = imu.GetBodyAngularRate();
            double yaw = std::get<2>(angularRate); // yaw from imu
            double setpoint = 0; // desired output  
            double error; // difference between setpoint and processVariable  
            double previousError = 0.09; // error in previous iteration  
            double integral ; // integral of error  
            double derivative; // derivative of error  
            double kp = 0.01; // proportional gain  
            double ki = 0.01; // integral gain  
            double kd = 0.0001; // derivative gain  
            double output; // output of the controller  
            double fsw_sim_time = 0.005; // time step (dt)
            double flight_time = 16; // time of entire flight
            double fsw_calibration_time = 10;
            double structures_I_hat_zz = 0.0031768;
            double wheel_omega;
            double wheel_I_hat_zz = 0.00007053;

			std::cout << "Angular Rate: " << yaw << "\n"; 
            int total_iterations = static_cast<int>(16.0 / fsw_sim_time);
            for (double i = 1; i <= total_iterations; ++i) {
                double processVariable = yaw; // current output from imu
                error = setpoint - processVariable;
                integral += error * fsw_sim_time;
                derivative = (error - previousError) / fsw_sim_time;
                output = kp * error + ki * integral + kd * derivative;
                previousError = error;
                processVariable = output;

                std::cout << "PID: " << output << std::endl;}// Print result
                std::this_thread::sleep_for(std::chrono::milliseconds(500));  //delay
		}
	}};

    std::this_thread::sleep_for(std::chrono::seconds(16));
    Exitloop = true;

    imuThread.join();
    return 0;}