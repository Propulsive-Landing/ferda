#include <string>
#include <fstream>
#include <chrono>
#include <stdio.h>

#include "Mode.hpp"
#include "Navigation.hpp"
#include "GPS.hpp"
#include "PressureTransducer.hpp"
#include "LoadCell.hpp"

class Telemetry
{
private:
    Telemetry();
    ~Telemetry();

    void HardwareSaveFrame(Navigation &navigation, Controller &controller, GPS &gps);
    void GPSSaveFrame(GPS &gps);
    void RfSendFrame(Navigation &navigation, Controller &controller, PressureTransducer &pt, LoadCell &lc);

public:
    std::chrono::steady_clock::time_point StartTime;

    std::ofstream Logs;
    std::ofstream HardwareSaved;
    std::ofstream SensorSaved;
    std::ofstream GPSSaved;

    void RunTelemetry(Navigation &navigation, Controller &controller, GPS &gps, PressureTransducer &pt, LoadCell &lc, float HardwareSaveDelta, float RFSaveDelta);
    void Log(std::string message);

    static Telemetry &GetInstance()
    {
        static Telemetry telem;

        return telem;
    }
};
