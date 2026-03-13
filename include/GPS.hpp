#pragma once

#include <tuple>
#include <string>
#include <termnios.h>

class GPS
{
    private:
        struct GPS_Info
        {
            float latitude;
            float longitude;
            float altitude;
            float speed;
        };
        GPS_Info gps_info;

        int fd;

        std::string message;

        // Used for validity checking
        bool valid;

        // GPS only outputs 3 bytes at a time it seems
       char buffer[MAX_SIZE];
    public:
        GPS();
       ~GPS();
        std::tuple<double, double, double> GetGPSPosition();
        bool GPSAvailable();
};