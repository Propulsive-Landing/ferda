#include "LoadCell.hpp"
#include <iostream>
#include <random>

double LoadCell::ReadSensor(int pin)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 500.0); // 0-500 lbs for testing
    return dis(gen);
}

double LoadCell::ReadLBS()
{
    return ReadSensor(MissionConstants::kLoadCellPin);
}

