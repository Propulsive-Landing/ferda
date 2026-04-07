// hardware_simulation/LoadCell.cpp
#include "LoadCell.hpp"
#include <iostream>
#include <random>

double LoadCell::ReadSensor(int pin)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 300.0); // 0-300 lbs for simulation
    return dis(gen);
}

double LoadCell::ReadLBS()
{
    return ReadSensor(MissionConstants::kLoadCellPin);
}

