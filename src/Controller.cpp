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
