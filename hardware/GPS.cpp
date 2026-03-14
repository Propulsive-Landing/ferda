// Update GPS class for hardware implementation

// hardware_simulation/GPS.cpp
#include "GPS.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <cmath>
#include "MissionConstants.hpp"


GPS::GPS()
{
    fd = open(MissionConstants::GPS_Port, O_RDWR);
    if (fd < 0)
    {
        std::cerr << "Error opening port" << "\n";
        exit(-1);
    }

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto str = oss.str();

    GPSReceived.open("../logs/GPSReceived" + str + ".txt");

    // Set gps_info values to a default -1
    gps_info.latitude = -1;
    gps_info.longitude = -1;
    gps_info.altitude = -1;
    gps_info.course = -1;
    gps_info.speed = -1;

    // Get rid of any garbage values o startup
    tcflush(fd, TCIFLUSH);
    valid = false;
}

GPS::~GPS()
{
    GPSReceived.close();
    int status = close(fd);
    if (status < 0)
    {
        std::cerr << "Error closing port" << "\n";
        exit(-1);
    }
    std::cout << " Here";
    
}

std::string GPS::get_message()
{
    return message;
}

std::vector<std::string> GPS::get_acculumated_messages()
{
    return acculumated_messages;
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
    std::cout << "Wrote " << settings << "to GPS" << "\n";
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

float GPS::convert_latitude(const std::string &latitude, const char &latitude_direction)
{
    std::string latitude_dd = latitude.substr(0, 2);
    std::string latitude_mmmmmm = latitude.substr(2, 7);
    float result =  stof(latitude_dd) + (stof(latitude_mmmmmm) / 60.0);
    if (latitude_direction == 'S')
    {
        result =  result * -1;
    }
    return result;
}
float GPS::convert_longitude(const std::string &longitude, const char &longitude_direction)
{
    std::string longitude_ddd = longitude.substr(0, 3);
    std::string longitude_mmmmmm = longitude.substr(3, 7);
    float result =  stof(longitude_ddd) + (stof(longitude_mmmmmm) / 60.0);
    if (longitude_direction == 'W')
    {
        result = result * -1;
    }
    return result;
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
        std::string status = nmea_message_parts[MissionConstants::NMEA::RMC::STATUS_IDX];
        char status_character = static_cast<char>(status[0]);
        if (status_character == MissionConstants::NMEA::RMC::BAD_STATUS_CHARACTER)
        {
            valid = false;
            std::cout << "Data is not valid, failed to update GPS_info" << "\n";
        }
        else
        {
            valid=true;
            parse_RMC(nmea_message_parts);
        }
    }
    else if (nmea_message_type == MissionConstants::NMEA::GGA::GGA)
    {
        std::cout << "Got GGA output type " << "\n";
        std::string status = nmea_message_parts[MissionConstants::NMEA::GGA::STATUS_IDX];
        int status_int = stoi(status);
        if (status_int == MissionConstants::NMEA::GGA::BAD_STATUS_NUMBER)
        {
            valid = false;
            std::cout << "Data is not valid, failed to update GPS_info" << "\n";
        }
        else
        {
            valid=true;
            parse_GGA(nmea_message_parts);
        }
    }
}

void GPS::parse_RMC(const std::vector<std::string> &nmea_message_parts)
{
    float time = stof(nmea_message_parts[MissionConstants::NMEA::TIME_IDX]);
    char latitude_direction = static_cast<char>(nmea_message_parts[MissionConstants::NMEA::RMC::LATITUDE_DIRECTION_IDX][0]);
    float latitude = convert_latitude(nmea_message_parts[MissionConstants::NMEA::RMC::LATITUDE_IDX], latitude_direction);
    char longitude_direction = static_cast<char>(nmea_message_parts[MissionConstants::NMEA::RMC::LONGITUDE_DIRECTION_IDX][0]);
    float longitude = convert_longitude(nmea_message_parts[MissionConstants::NMEA::RMC::LONGITUDE_IDX], longitude_direction);

    float course = stof(nmea_message_parts[MissionConstants::NMEA::RMC::COURSE_IDX]);
    float speed = convert_speed_to_meter_per_seconds(nmea_message_parts[MissionConstants::NMEA::RMC::SPEED_IDX]);

    // ADRESS ISSUE OF ENSURING TIMES MATCH UP FOR THE DIFFERENT NEMA SENTENCE TYPES

    // Sanity check:
    std::cout << "Time: " << time << "\n";
    std::cout << "Latitude: " << latitude << "\n";
    std::cout << "Longitude : " << longitude << "\n";
    std::cout << "Course: " << course << "\n";
    std::cout << "Speed: " << speed << "\n";
    std::cout << "\n";

    gps_info.latitude = latitude * MissionConstants::kDeg2Rad;;
    gps_info.longitude = longitude * MissionConstants::kDeg2Rad;;
    gps_info.speed = speed;
    gps_info.course = course;
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
    float lat_init = 41.808956  * MissionConstants::kDeg2Rad;
    float long_init = 72.255743 * MissionConstants::kDeg2Rad;
    float alttiude_init = 0;
    float dphi = gps_info.latitude - lat_init;
    float dlambda = gps_info.longitude  - long_init;
    float dh = gps_info.altitude - alttiude_init;

    float earth_radius = 6378137;
    float RN = earth_radius;
    float RM = earth_radius;

    float E = (RN + alttiude_init)*std::cos(lat_init)*dlambda;
    float N = (RM + alttiude_init) * dphi;
    float U = dh; 

    // std::cout << std::cos(lat_init)*dlambda<< "\n";
    // std::cout << gps_info.latitude << ", " << gps_info.longitude << "\n";
    // std::cout << E << ", " << N << "\n";
    return std::make_tuple(E, N, 0);
}

// void GPS::read_data()
// {

// }

void GPS::read_data()
{

    auto now = std::chrono::system_clock::now();

    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();

    std::time_t tt = std::chrono::system_clock::to_time_t(seconds);

    std::tm tm;
    localtime_r(&tt, &tm);  // thread-safe on Linux

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms;

    // It seems to only send 3 bytes at at time; Might have to do with frequency but probably not
    // Either way, this code fully reads the message and it will only break if buffer exceeds the max size of buffer
    int bytes_received = read(fd, buffer, sizeof(buffer));
    std::cout << "Received " << bytes_received << "\n";
    for (int i = 0; i < bytes_received; ++i)
    {
        char character = buffer[i];
        message += character;
        // this->GPSReceived << message << std::flush;
        // return message;

        // If we are at the end of a NMEA message, let's first make sure that we recieved a full message by trying to find '$'
        if (character == '\n')
        {
            if (message.find('$') != std::string::npos)
            {
                // Print message for sanity check
                //std::cout << message;
                acculumated_messages.push_back(message);
                 this->GPSReceived << oss.str() << ", " << message << std::flush;
                message.clear();
                // TODO THIS HAS TO BE SEPARATE
                // std::vector<std::string> nmea_message_parts = break_message_down(message);
                // std::string NMEA_type = determine_NMEA_type(nmea_message_parts);
                // parse_NMEA_type(NMEA_type, nmea_message_parts);
            }
        }
    }

    memset(buffer, 0, bytes_received);
}

bool GPS::GPSAvailable()
{
    return valid;
}

void GPS::set_valid(const bool state)
{
    valid = state;
}
