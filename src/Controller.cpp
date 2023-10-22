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

// Handle whichever abort gets called
void Controller::HandleAborts(int abort) {
    switch(abort) {
        case 1:
            // code
            break;
        case 2:
            // code
            break;
        case 3:
            // code
            break;
        case 4:
            // code
            break;
        default:
            break;
    }
}
