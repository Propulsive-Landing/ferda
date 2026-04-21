#pragma once

#ifdef NDEBUG
float readPositionInches(int actuator_index);
float readPositionInches();
void driveActuator(int actuator_index, int direction, int speed);
int moveToLimit(int actuator_index, int direction);
void RunChirpTVCMode();
#else
inline void RunChirpTVCMode() {}
#endif