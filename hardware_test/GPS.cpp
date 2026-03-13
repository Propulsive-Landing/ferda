// Update GPS class for hardware implementation

// hardware_simulation/GPS.cpp
#include "GPS.hpp"

GPS::GPS() {}

GPS::~GPS() {}

void GPS::write_settings(const std::string &settings)
{
}

std::vector<std::string> GPS::break_message_down(const std::string &message)
{
}

float GPS::convert_latitude(const std::string &latitude)
{
    return 0.0f;
}
float GPS::convert_longitude(const std::string &longitude)
{
    return 0.0f;
}

float GPS::convert_speed_to_meter_per_seconds(const std::string &speed)
{
    return 0.0f;
}

std::string GPS::determine_NMEA_type(const std::vector<std::string> &nmea_message_parts)
{
    // Return which NMEA outout was used
    return std::string("");
}

void GPS::parse_NMEA_type(const std::string nmea_message_type, const std::vector<std::string> &nmea_message_parts)
{
}

void GPS::parse_RMC(const std::vector<std::string> &nmea_message_parts)
{
}

void GPS::parse_GGA(const std::vector<std::string> &nmea_message_parts)
{
}

std::tuple<double, double, double> GPS::GetGPSPosition()
{
    return std::make_tuple(gps_info.latitude, gps_info.longitude, gps_info.altitude);
}

void GPS::read_data()
{
}

bool GPS::GPSAvailable()
{
    return valid;
}
