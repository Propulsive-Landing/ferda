#include <tuple>

#include "IMU.hpp"

#include "SimulationManager.hpp"

std::tuple<double, double, double> IMU::GetBodyAngularRate()
{

    Eigen::Matrix<double, 7, 1> Outputs = SimulationManager::GetInstance().GetOutputs();


    return std::make_tuple(Outputs(0, 0), Outputs(1, 0), Outputs(2, 0));
}

std::tuple<double, double, double> IMU::GetBodyAcceleration()
{
    return std::make_tuple(0.0,0.0,0.0);
}
