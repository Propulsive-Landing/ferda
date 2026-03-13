// Update GPS class for hardware implementation

// hardware_simulation/GPS.cpp
#include "GPS.hpp"
#include <fcntl.h>
#include <unistd.h>

GPS::GPS()
{
    fd = open(MissionConstants::GPS_Port, O_RDWR);
    if (fd < 0)
    {
        std::cerr << "Error opening port" << "\n";
        exit(-1);
    }
    // Get rid of any garbage values o startup
    tcflush(fd, TCIFLUSH);
    valid = true;
}

GPS::~GPS()
{
    int status = close(fd);
    if (status < 0)
    {
        std::cerr << "Error closing port" << "\n";
        exit(-1);
    }
}

void GPS::write_settings(const std::string &settings)
{
    // Flush input buffer so that the chances of reading the our message is better
    // Currentely, we don't do anything if this does not work since we only look for the certain NMEA types anyway
    tcflush(fd, TCIFLUSH);
    int status = write(fd, settings.c_str(), settings.size());
    if (status < 0)
    {
        std::cerr << "Error writing to fd" << "\n";
        exit(-1);
    }
    std::cout << "Wrote " << settings << "to GPS";
}

std::vector<std::string> GPS::break_message_down(const std::string &message)
{
    // Break down NMEA output message into subparts because it is easer to find validity part of sentence
    // RMC validity is always one of the first ones but GGA validity is midway so I can't reliable use indexes yet
    // It's better to extract all info from commas and then go by position that way
    std::vector<std::string> nmea_message_parts;
    int curr_pos = MissionConstants::NMEA::MESSAGE_TYPE_STARTING_STRING_INDEX;
    std::string substring;
    int pos = message.find(',', curr_pos);

    while (pos != std::string::npos)
    {
        substring = message.substr(curr_pos, pos - curr_pos);
        nmea_message_parts.push_back(substring);
        curr_pos = pos + 1;
        pos = message.find(',', curr_pos);
    }
    // Add last part
    substring = message.substr(curr_pos, message.length() - curr_pos);
    nmea_message_parts.push_back(substring);

    return nmea_message_parts;
}

float GPS::convert_latitude(const std::string &latitude)
{
    std::string latitude_dd = latitude.substr(0, 2);
    std::string latitude_mmmmmm = latitude.substr(2, 7);
    return stof(latitude_dd) + stof(latitude_mmmmmm) / 60.0;
}
float GPS::convert_longitude(const std::string &longitude)
{
    std::string longitude_ddd = longitude.substr(0, 3);
    std::string longitude_mmmmmm = longitude.substr(3, 7);
    return stof(longitude_ddd) + stof(longitude_mmmmmm) / 60.0;
}

float GPS::convert_speed_to_meter_per_seconds(const std::string &speed)
{
    return stof(speed) * 0.514444;
}

std::string GPS::determine_NMEA_type(const std::vector<std::string> &nmea_message_parts)
{
    // Return which NMEA outout was used
    return nmea_message_parts[MissionConstants::NMEA::MESSAGE_TYPE_IDX];
}

void GPS::parse_NMEA_type(const std::string nmea_message_type, const std::vector<std::string> &nmea_message_parts)
{
    if (nmea_message_type == MissionConstants::NMEA::RMC::RMC)
    {
        std::cout << "Got RMC output type " << "\n";
        std::string status = nmea_message_parts[MissionConstants::NMEA::RMC::STAUS_IDX];
        char status_character = static_cast<char>(status[0]);
        if (status_character == MissionConstants::NMEA::RMC::BAD_STATUS_CHARACTER)
        {
            valid = false;
            std::cout << "Data is not valid, failed to update GPS_info" << "\n";
        }
        else
        {
            parse_RMC(nmea_message_parts);
        }
    }
    else if (nmea_message_type == MissionConstants::NMEA::GGA::GGA)
    {
        std::cout << "Got GGA output type " << "\n";
        std::string status = nmea_message_parts[MissionConstants::NMEA::GGA::STAUS_IDX];
        int status_int = stoi(status);
        if (status_int == MissionConstants::NMEA::GGA::BAD_STATUS_NUMBER)
        {
            valid = false;
            std::cout << "Data is not valid, failed to update GPS_info" << "\n";
        }
        else
        {
            parse_GGA(nmea_message_parts);
        }
    }
}

void GPS::parse_RMC(const std::vector<std::string> &nmea_message_parts)
{
    float time = stof(nmea_message_parts[MissionConstants::NMEA::TIME_IDX]);
    float latitude = convert_latitude(nmea_message_parts[MissionConstants::NMEA::RMC::LATITUDE_IDX]);
    float longitude = convert_longitude(nmea_message_parts[MissionConstants::NMEA::RMC::LONGITUDE_IDX]);
    float speed = convert_speed_to_meter_per_seconds(nmea_message_parts[MissionConstants::NMEA::RMC::SPEED_IDX]);

    // ADRESS ISSUE OF ENSURING TIMES MATCH UP FOR THE DIFFERENT NEMA SENTENCE TYPES

    // Sanity check:
    std::cout << "Time: " << time << "\n";
    std::cout << "Latitude: " << latitude << "\n";
    std::cout << "Longitude : " << longitude << "\n";
    std::cout << "Speed: " << speed << "\n";
    std::cout << "\n";

    gps_info.latitude = latitude;
    gps_info.longitude = longitude;
    gps_info.speed = speed;
}

void GPS::parse_GGA(const std::vector<std::string> &nmea_message_parts)
{
    float time = stof(nmea_message_parts[MissionConstants::NMEA::TIME_IDX]);
    float altitude = stof(nmea_message_parts[MissionConstants::NMEA::GGA::ALTITUDE_INDEX]);

    // ADRESS ISSUE OF ENSURING TIMES MATCH UP FOR THE DIFFERENT NEMA SENTENCE TYPES

    // Sanity check:
    std::cout << "Time: " << time << "\n";
    std::cout << "Altitude: " << altitude << "\n";
    std::cout << "\n";

    gps_info.altitude = altitude;
}

std::tuple<double, double, double> GPS::GetGPSPosition()
{
    return std::make_tuple(gps_info.latitude, gps_info.longitude, gps_info.altitude);
}

void GPS::read_data()
{
    // It seems to only send 3 bytes at at time; Might have to do with frequency but probably not
    // Either way, this code fully reads the message and it will only break if buffer exceeds the max size of buffer
    int bytes_received = read(fd, buffer, sizeof(buffer));
    // std::cout << "Received " << bytes_received << "\n";
    for (int i = 0; i < bytes_received; ++i)
    {
        char character = buffer[i];
        message += character;

        // If we are at the end of a NMEA message, let's first make sure that we recieved a full message by trying to find '$'
        if (character == '\n')
        {
            if (message.find('$') != std::string::npos)
            {
                // Print message for sanity check
                std::cout << message;
                std::vector<std::string> nmea_message_parts = break_message_down(message);
                std::string NMEA_type = determine_NMEA_type(nmea_message_parts);
                parse_NMEA_type(NMEA_type, nmea_message_parts);
            }
            message.clear();
        }
    }

    memset(buffer, 0, bytes_received);
}

bool GPS::GPSAvailable()
{
    return valid;
}
