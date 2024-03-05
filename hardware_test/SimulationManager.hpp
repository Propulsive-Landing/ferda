#pragma once

#include <cstdint>
#include <Eigen/Dense>

class SimulationManager
{
private:
    SimulationManager();
    ~SimulationManager();

    int clientSocket;
    bool validConnection; // True if connected successfully

    const float POLL_TIME = 0.1;


    // P, Q, R, x_accel, y_accel, z_accel, pressure
    Eigen::Matrix<double, 8, 1> SimulationOutputs;
    
    // tvc_x, tvc_y, ignite1, ignite2
    Eigen::Matrix<double, 4, 1> SimInputs;


    const char *hostname = "8.tcp.ngrok.io";
    uint16_t port = 15977;

public:
    

    Eigen::Matrix<double, 8, 1> GetOutputs();
    void SetInputs(Eigen::Matrix<double, 4, 1> inputs);
    Eigen::Matrix<double, 4, 1> GetCurrentInputs();

    static SimulationManager& GetInstance()
    {
        static SimulationManager telem;

        return telem;
    } 

};
