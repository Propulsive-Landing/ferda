#include "Barometer.hpp"

#include "SimulationManager.hpp"

double Barometer::GetPressure()
{
    Eigen::Matrix<double, 8, 1> Outputs = SimulationManager::GetInstance().GetOutputs();

    return Outputs(6, 0);
}

double Barometer::GetTemperature()
{
    Eigen::Matrix<double, 8, 1> Outputs = SimulationManager::GetInstance().GetOutputs();

    return Outputs(7, 0);
}
