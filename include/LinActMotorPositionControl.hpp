#pragma once

#ifdef NDEBUG
#include <memory>
#include "PCA9685Driver.hpp"
#endif

float readPositionInches(int actuator_index);
float readPositionInches();
void driveActuator(int actuator_index, int direction, int speed);
int moveToLimit(int actuator_index, int direction);
void RunChirpTVCMode();
