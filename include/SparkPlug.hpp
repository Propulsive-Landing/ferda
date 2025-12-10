#pragma once

#include "MissionConstants.hpp"

class SparkPlug
{
public:
    SparkPlug() = default;
    void TurnOn();
    void TurnOff();
    bool IsOn() const;

private:
    bool isOn = false;
};

