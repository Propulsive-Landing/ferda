#include <Eigen/Dense>

#include "Controller.hpp"


Controller::Controller(TVC& tvc, Igniter& igniter) : tvc(tvc), igniter(igniter) {}


void Controller::UpdateIdle(Navigation& navigation) {
    Eigen::Matrix<double, 12, 1> x = navigation.GetNavigation();
    // TODO. Calculate desired control inputs for ground
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