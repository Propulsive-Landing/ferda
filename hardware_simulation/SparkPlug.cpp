// hardware_simulation/SparkPlug.cpp
#include "SparkPlug.hpp"
#include <iostream>

void SparkPlug::TurnOn()
{
    std::cout << "[SIM] Spark plug turned ON" << std::endl;
    isOn = true;
}

void SparkPlug::TurnOff()
{
    std::cout << "[SIM] Spark plug turned OFF" << std::endl;
    isOn = false;
}

bool SparkPlug::IsOn() const
{
    return isOn;
}

