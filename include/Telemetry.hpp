#include <string>
#include <fstream>
#include <stdio.h>

#include "Mode.hpp"
#include "Navigation.hpp"
#include "PressureTransducer.hpp"
#include "LoadCell.hpp"

class Telemetry
{
private:
    Telemetry();
    ~Telemetry();

    void HardwareSaveFrame(Navigation &navigation, Controller &controller);
    void RfSendFrame(Navigation &navigation, Controller &controller);
    void RfSendLiquidPropulsionData(PressureTransducer &pt, LoadCell &lc);

public:
    std::ofstream Logs;
    std::ofstream HardwareSaved;
    std::ofstream SensorSaved;

    void RunTelemetry(Navigation &navigation, Controller &controller, PressureTransducer &pt, LoadCell &lc, float HardwareSaveDelta, float RFSaveDelta);
    void Log(std::string message);

    static Telemetry &GetInstance()
    {
        static Telemetry telem;

        return telem;
    }
};
