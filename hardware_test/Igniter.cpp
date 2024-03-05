#include "Igniter.hpp"
#include "SimulationManager.hpp"

#include <Eigen/Dense>

void Igniter::Ignite(Igniter::IgnitionSpecifier ignite)
{
    Eigen::Matrix<double, 4, 1> CurrentInput = SimulationManager::GetInstance().GetCurrentInputs();

    if(ignite == Igniter::LAUNCH){
        CurrentInput(2, 0) = 1;
    }
    else if(ignite == Igniter::LAND){
        CurrentInput(3, 0) = 1;
    }


    SimulationManager::GetInstance().SetInputs(CurrentInput); // IDK if this is redundant but It shouldn't effect anything. TODO

    
}