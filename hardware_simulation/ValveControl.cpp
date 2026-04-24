// hardware_simulation/ValveControl.cpp
#include "ValveControl.hpp"
#include <iostream>

void ValveControl::OpenValve(ValveType valve)
{
    switch (valve)
    {
    case Nitrogen:
        std::cout << "[SIM] Nitrogen valve OPENED" << std::endl;
        nitrogenOpen = true;
        break;
    case Purge:
        std::cout << "[SIM] Purge valve OPENED" << std::endl;
        purgeOpen = true;
        break;
    case MainEthanol:
        std::cout << "[SIM] Main Ethanol valve OPENED" << std::endl;
        mainEthanolOpen = true;
        break;
    case MainNitrous:
        std::cout << "[SIM] Main Nitrous valve OPENED" << std::endl;
        mainNitrousOpen = true;
        break;
    case NitrousFill:
        std::cout << "[SIM] Nitrous Fill valve OPENED" << std::endl;
        nitrousFillOpen = true;
        break;
    case ASIEthanol:
        std::cout << "[SIM] ASI Ethanol valve OPENED" << std::endl;
        asiEthanolOpen = true;
        break;
    case ASIOxygen:
        std::cout << "[SIM] ASI Oxygen valve OPENED" << std::endl;
        asiOxygenOpen = true;
        break;
    case NitrogenBleed:
        std::cout << "[SIM] Nitrogen Bleed valve OPENED" << std::endl;
        nitrogenBleedOpen = true;
        break;
    }
}

void ValveControl::CloseValve(ValveType valve)
{
    switch (valve)
    {
    case Nitrogen:
        std::cout << "[SIM] Nitrogen valve CLOSED" << std::endl;
        nitrogenOpen = false;
        break;
    case Purge:
        std::cout << "[SIM] Purge valve CLOSED" << std::endl;
        purgeOpen = false;
        break;
    case MainEthanol:
        std::cout << "[SIM] Main Ethanol valve CLOSED" << std::endl;
        mainEthanolOpen = false;
        break;
    case MainNitrous:
        std::cout << "[SIM] Main Nitrous valve CLOSED" << std::endl;
        mainNitrousOpen = false;
        break;
    case NitrousFill:
        std::cout << "[SIM] Nitrous Fill valve CLOSED" << std::endl;
        nitrousFillOpen = false;
        break;
    case ASIEthanol:
        std::cout << "[SIM] ASI Ethanol valve CLOSED" << std::endl;
        asiEthanolOpen = false;
        break;
    case ASIOxygen:
        std::cout << "[SIM] ASI Oxygen valve CLOSED" << std::endl;
        asiOxygenOpen = false;
        break;
    case NitrogenBleed:
        std::cout << "[SIM] Nitrogen Bleed valve CLOSED" << std::endl;
        nitrogenBleedOpen = false;
        break;
    }
}

bool ValveControl::IsValveOpen(ValveType valve) const
{
    switch (valve)
    {
    case Nitrogen:
        return nitrogenOpen;
    case Purge:
        return purgeOpen;
    case MainEthanol:
        return mainEthanolOpen;
    case MainNitrous:
        return mainNitrousOpen;
    case NitrousFill:
        return nitrousFillOpen;
    case ASIEthanol:
        return asiEthanolOpen;
    case ASIOxygen:
        return asiOxygenOpen;
    case NitrogenBleed:
        return nitrogenBleedOpen;
    default:
        return false;
    }
}
