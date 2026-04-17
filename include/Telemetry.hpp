#include <string>
#include <fstream>
#include <chrono>
#include <stdio.h>

#include "Mode.hpp"
#include "Navigation.hpp"
#include "GPS.hpp"

class Telemetry
{
private:
    Telemetry();
    ~Telemetry();

    void HardwareSaveFrame(Navigation &navigation, Controller &controller, GPS &gps);
    void GPSSaveFrame(GPS &gps);
    void RfSendFrame(Navigation &navigation, Controller &controller);
    void LogActuatorFrame(double commanded_angle_x_rad,
                          double commanded_angle_y_rad,
                          double commanded_length_x_in,
                          double commanded_length_y_in,
                          double observed_length_x_in,
                          double observed_length_y_in,
                          int commanded_speed_x,
                          int commanded_speed_y);

public:
    std::chrono::steady_clock::time_point StartTime;

    std::ofstream Logs;
    std::ofstream HardwareSaved;
    std::ofstream SensorSaved;
    std::ofstream GPSSaved;
    std::ofstream ActuatorSaved;

    void RunTelemetry(Navigation &navigation, Controller &controller, GPS &gps, float HardwareSaveDelta, float RFSaveDelta);
    void Log(std::string message);

    static Telemetry &GetInstance()
    {
        static Telemetry telem;

        return telem;
    }
};
