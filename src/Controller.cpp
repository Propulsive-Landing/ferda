#include <Eigen/Dense>

#include "Controller.hpp"
#include "TVC.hpp"
#include "Igniter.hpp"


Controller::Controller(TVC& tvc, Igniter& igniter) : tvc(tvc), igniter(igniter)
{
}

void Controller::ignite(){

}

void Controller::launch(){

}

//listen for ground control commands
void Controller::updateGround(){

}


void Controller::updateLaunch(){

}

//communicate with TVC
void Controller::updateLand(){

}

//shut down rocket functions
void Controller::updateSafe(){

}