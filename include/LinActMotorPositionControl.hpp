#pragma once

#ifdef NDEBUG
#include <memory>
#include <PCA9685Driver.h>
float readPositionInches(int actuator_index);
float readPositionInches();
void driveActuator(int actuator_index, int direction, int speed);
void RunChirpTVCMode();
#else
inline void RunChirpTVCMode() {}
#endif