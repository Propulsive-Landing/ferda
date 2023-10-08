#include "Controller.hpp"
#include <vector>

//possibly add navigator later
Controller::Controller(TVC tvc, Igniter igniter){
    this.tvc = tvc;
    this.igniter = igniter

    vector<double> IMU;
    
}


void Controller::setTVCservos(doulbe X, double Y){

}

std::vector<double> Controller::getIMUData(){

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