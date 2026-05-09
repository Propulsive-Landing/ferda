#pragma once

#include "MissionConstants.hpp"

class LoadCell
{
public:
    LoadCell() = default;
    double ReadLBS(); // Returns force in pounds

private:
    double ReadSensor(int pin); // Internal helper
};

