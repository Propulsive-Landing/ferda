#include <vector>
#include "Controller.hpp"
#include "TVC.hpp"
#include "Igniter.hpp"

//possibly add navigator later
Controller::Controller(TVC tvc, Igniter igniter){
    this->tvc = tvc;
    this->igniter = igniter;

    std::vector<double> IMU;
    
}


void Controller::setTVCservos(double X, double Y){

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