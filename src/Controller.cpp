#include <Eigen/Dense>

#include "Controller.hpp"


Controller::Controller(TVC& tvc, Igniter& igniter) : tvc(tvc), igniter(igniter)
{
}

void Controller::ignite(){

}

void Controller::launch(){

}

//listen for ground control commands
void Controller::UpdateGround(){

}


void Controller::UpdateLaunch(Navigation& navigation){

}

//communicate with TVC
void Controller::UpdateLand(){

}

//shut down rocket functions
void Controller::UpdateSafe(){

}