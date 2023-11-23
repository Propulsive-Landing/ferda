#include <Eigen/Dense>

#include "Controller.hpp"


Controller::Controller(TVC& tvc, Igniter& igniter) : tvc(tvc), igniter(igniter) {}


void Controller::UpdateGround(Navigation& navigation) {
    // TODO. Calculate desired control inputs for ground
    navigation.GetNavigation();
    // TODO. Actuate all control surfaces accordingly
}

void Controller::UpdateLaunch(Navigation& navigation) {
    // TODO. Calculate desired control inputs for launch
    // TODO. Actuate all control surfaces accordingly
}

//communicate with TVC
void Controller::UpdateLand(){
    // TODO. Calculate desired control inputs for land
    // TODO. Actuate all control surfaces accordingly
}

//shut down rocket functions
void Controller::UpdateSafe(){
    // TODO. Center TVC, turn off reaction wheel, etc.
}