#include "TVC.hpp"
#include "SimulationManager.hpp"

#include <Eigen/Dense>

void TVC::SetXServo(double dAngle)
{
    Eigen::Matrix<double, 4, 1> CurrentInput = SimulationManager::GetInstance().GetCurrentInputs();
    CurrentInput(0, 0) = dAngle;

    SimulationManager::GetInstance().SetInputs(CurrentInput); // IDK if this is redundant but It shouldn't effect anything. TODO
}

void TVC::SetYServo(double dAngle)
{
    Eigen::Matrix<double, 4, 1> CurrentInput = SimulationManager::GetInstance().GetCurrentInputs();
    CurrentInput(0, 1) = dAngle;

    SimulationManager::GetInstance().SetInputs(CurrentInput); // IDK if this is redundant but It shouldn't effect anything. TODO
}