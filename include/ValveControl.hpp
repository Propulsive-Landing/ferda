#pragma once

#include "MissionConstants.hpp"

class ValveControl
{
public:
    enum ValveType
    {
        Nitrogen,
        Purge,
        MainEthanol,
        MainNitrous,
        ASIEthanol,
        ASIOxygen,
        NitrogenBleed
    };

    ValveControl() = default;
    void OpenValve(ValveType valve);
    void CloseValve(ValveType valve);
    bool IsValveOpen(ValveType valve) const;
    float convert_to_ticks(float angle)
    {
        // 1500 us is nuetral position (90 degrees)
        // Figures out duty cycle with respect to the period
        float pulse = 1500 + ((angle - 90) / 90.0) * 1000;
        return (pulse / MissionConstants::SERVO_PERIOD) * MissionConstants::MAX_TICKS;
    }

private:
    bool nitrogenOpen = false;
    bool purgeOpen = false;
    bool mainEthanolOpen = false;
    bool mainNitrousOpen = false;
    bool asiEthanolOpen = false;
    bool asiOxygenOpen = false;
    bool nitrogenBleedOpen = false;
};
