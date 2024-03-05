#include <tuple>

#include "IMU.hpp"

#include "SimulationManager.hpp"

std::tuple<double, double, double> IMU::GetBodyAngularRate()
{

    Eigen::Matrix<double, 8, 1> Outputs = SimulationManager::GetInstance().GetOutputs();

    return std::make_tuple(Outputs(0, 0), Outputs(1, 0), Outputs(2, 0));
}

std::tuple<double, double, double> IMU::GetBodyAcceleration()
{
    Eigen::Matrix<double, 8, 1> Outputs = SimulationManager::GetInstance().GetOutputs();

    return std::make_tuple(Outputs(3, 0), Outputs(4, 0), Outputs(5, 0));
}
