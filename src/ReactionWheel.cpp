#include <iostream>
#include <cmath>
#include <array>

    double setpoint = 0; // desired output  
    double processVariable = 0.09; // current output  
    double error; // difference between setpoint and processVariable  
    double previousError = 0.09; // error in previous iteration  
    double integral ; // integral of error  
    double derivative; // derivative of error  
    double kp = 0.01; // proportional gain  
    double ki = 0.01; // integral gain  
    double kd = 0.0001; // derivative gain  
    double output; // output of the controller  
    double fsw_sim_time = 0.005; // time step
    double fsw_calibration_time = 10;
    double structures_I_hat_zz = 0.0031768;
    double wheel_omega;
    double wheel_I_hat_zz = 0.00007053;
    
    int main() {
        for (double i = 1; i <= 3600; ++i) {
            double calculateOutput(double setpoint, double processVariable);
            error = setpoint - processVariable;
            integral += error * fsw_sim_time;
            derivative = (error - previousError) / fsw_sim_time;
            output = kp * error + ki * integral + kd * derivative;
            previousError = error;
            processVariable = output;

            // Print the result
            std::cout << "PID: " << error << std::endl;}

            return 0;}