#pragma once

#include <tuple>
#include <string>
#include <termios.h>
#include "MissionConstants.hpp"

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
    char buffer[MissionConstants::MAX_SIZE];

public:
    GPS();
    ~GPS();
    std::string determine_NMEA_type(const std::vector<std::string> &nmea_message_parts);
    void parse_NMEA_type(const std::string nmea_message_type, const std::vector<std::string> &nmea_message_parts);
    std::vector<std::string> break_message_down(const std::string &message);
    void parse_RMC(const std::vector<std::string> &message);
    void parse_GGA(const std::vector<std::string> &message);
    void read_data();
    void write_settings(const std::string &settings);
    float convert_latitude(const std::string &latitude);
    float convert_longitude(const std::string &longitude);
    float convert_speed_to_meter_per_seconds(const std::string &speed);
    std::tuple<double, double, double> GetGPSPosition();
    bool GPSAvailable();
};