#include "SparkPlug.hpp"
#include <iostream>

void SparkPlug::TurnOn()
{
    std::cout << "Spark plug turned ON" << std::endl;
    isOn = true;
}

void SparkPlug::TurnOff()
{
    std::cout << "Spark plug turned OFF" << std::endl;
    isOn = false;
}

bool SparkPlug::IsOn() const
{
    return isOn;
}

