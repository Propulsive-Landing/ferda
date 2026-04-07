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

private:
    bool nitrogenOpen = false;
    bool purgeOpen = false;
    bool mainEthanolOpen = false;
    bool mainNitrousOpen = false;
    bool asiEthanolOpen = false;
    bool asiOxygenOpen = false;
    bool nitrogenBleedOpen = false;
};

