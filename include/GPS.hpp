#pragma once

#include <cstdint>
#include <tuple>
#include <string>
#include <termios.h>
#include <fstream>
#include <map>
#include <vector>
#include "MissionConstants.hpp"

class GPS
{
private:
    struct GPS_Info
    {
        double latitude;
        double longitude;
        double E;
        double N;
        double U;
        double E_velocity;
        double N_velocity;
        double altitude;
        double course;
        double speed;
    };
    GPS_Info gps_info;

    int fd;

    std::ofstream GPSReceived;

    std::string message;

    std::vector<std::string> acculumated_messages;

    // Used for validity checking
    bool valid;
    bool fresh_position;
    bool fresh_velocity;
    bool has_last_rmc_time;
    bool has_last_gga_time;
    double last_rmc_time;
    double last_gga_time;

    uint64_t update_count;

    // Used as a flag to see if GPS is plugged in
    bool found_gps;

    char buffer[MissionConstants::MAX_SIZE];

    std::string get_message();
    std::vector<std::string> get_acculumated_messages();
    void reset_acculumated_messages();
    std::string determine_NMEA_type(const std::vector<std::string> &nmea_message_parts);
    bool parse_NMEA_type(const std::string nmea_message_type, const std::vector<std::string> &nmea_message_parts);
    std::vector<std::string> break_message_down(const std::string &message);
    bool parse_RMC(const std::vector<std::string> &message);
    bool parse_GGA(const std::vector<std::string> &message);
    void read_data();
    float convert_latitude(const std::string &latitude, const char &latitude_direction);
    float convert_longitude(const std::string &longitude, const char &longitude_direction);
    float convert_speed_to_meter_per_seconds(const std::string &speed);
    void convert_speed_course_to_velocity();
    void convert_coordinate_frame();
    void set_valid(bool state);

public:
    GPS();
    ~GPS();
    void Update();
    std::tuple<double, double, double> GetGPSPosition();
    std::tuple<double, double> GetGPSVelocity();
    bool GPSAvailable();
    bool GPSUsed();
    bool HasFreshPosition() const;
    bool HasFreshVelocity() const;
    uint64_t GetUpdateCount() const;
};