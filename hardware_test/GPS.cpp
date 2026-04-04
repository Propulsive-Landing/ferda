// Update GPS class for hardware implementation

// hardware_simulation/GPS.cpp
#include "GPS.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <set>
#include <chrono>
#include <iomanip>
#include <cmath>
#include "MissionConstants.hpp"

GPS::GPS()
{
    // Set gps_info values to defaults
    gps_info.latitude = -1;
    gps_info.longitude = -1;
    gps_info.altitude = -1;
    gps_info.E = -1;
    gps_info.N = -1;
    gps_info.U = -1;
    gps_info.course = -1;
    gps_info.speed = -1;
    gps_info.E_velocity = -1;
    gps_info.N_velocity = -1;

    fd = open(MissionConstants::GPS_Port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        std::cerr << "Warning: GPS port unavailable in test mode" << "\n";
        return;
    }

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto str = oss.str();

    GPSReceived.open("../logs/GPSReceived" + str + ".txt");

    // Get rid of any garbage values o startup
    tcflush(fd, TCIFLUSH);
    valid = false;
}

GPS::~GPS()
{
    GPSReceived.close();
    if (fd >= 0)
    {
        int status = close(fd);
        if (status < 0)
        {
            std::cerr << "Error closing port" << "\n";
            exit(-1);
        }
    }
}

std::string GPS::get_message()
{
    return message;
}

std::vector<std::string> GPS::get_acculumated_messages()
{
    return acculumated_messages;
}

void GPS::reset_acculumated_messages()
{
    acculumated_messages.clear();
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
    float result = stof(latitude_dd) + (stof(latitude_mmmmmm) / 60.0);
    if (latitude_direction == 'S')
    {
        result = result * -1;
    }
    return result;
}
float GPS::convert_longitude(const std::string &longitude, const char &longitude_direction)
{
    std::string longitude_ddd = longitude.substr(0, 3);
    std::string longitude_mmmmmm = longitude.substr(3, 7);
    float result = stof(longitude_ddd) + (stof(longitude_mmmmmm) / 60.0);
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
    if (valid && nmea_message_type == MissionConstants::NMEA::RMC::RMC)
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
            parse_RMC(nmea_message_parts);
        }
    }
    else if (valid && nmea_message_type == MissionConstants::NMEA::GGA::GGA)
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

    // Sanity check:
    std::cout << "Time: " << time << "\n";
    std::cout << "Latitude: " << latitude << "\n";
    std::cout << "Longitude : " << longitude << "\n";
    std::cout << "Course: " << course << "\n";
    std::cout << "Speed: " << speed << "\n";
    std::cout << "\n";

    gps_info.latitude = latitude * MissionConstants::kDeg2Rad;

    gps_info.longitude = longitude * MissionConstants::kDeg2Rad;

    gps_info.speed = speed;
    gps_info.course = course;
    convert_speed_course_to_velocity();
}

void GPS::parse_GGA(const std::vector<std::string> &nmea_message_parts)
{
    float time = stof(nmea_message_parts[MissionConstants::NMEA::TIME_IDX]);
    float altitude = stof(nmea_message_parts[MissionConstants::NMEA::GGA::ALTITUDE_INDEX]);

    // Sanity check:
    std::cout << "Time: " << time << "\n";
    std::cout << "Altitude: " << altitude << "\n";
    std::cout << "\n";

    gps_info.altitude = altitude;
}

std::tuple<double, double, double> GPS::GetGPSPosition()
{
    return std::make_tuple(gps_info.E, gps_info.N, gps_info.U);
}

std::tuple<double, double> GPS::GetGPSVelocity()
{
    return std::make_tuple(gps_info.E_velocity, gps_info.N_velocity);
}

void GPS::convert_coordinate_frame()
{
    static double lat_init = gps_info.latitude;
    static double long_init = gps_info.longitude;
    static double altitude_init = gps_info.altitude;

    double dphi = gps_info.latitude - lat_init;
    double dlambda = gps_info.longitude - long_init;
    double dh = gps_info.altitude - altitude_init;

    double earth_radius = 6378137;
    double RN = earth_radius;
    double RM = earth_radius;

    float E = (RN + altitude_init) * std::cos(lat_init) * dlambda;
    float N = (RM + altitude_init) * dphi;
    float U = dh;

    gps_info.E = E;
    gps_info.N = N;
    gps_info.U = U;

    std::cout << std::cos(lat_init) * dlambda << "\n";
    std::cout << "Latitude init: " << lat_init << "\n";

    std::cout << "Longitude init: " << long_init << "\n";

    std::cout << "Altitude init: " << altitude_init << "\n";

    std::cout << "DPHI: " << dphi << "\n";

    std::cout << gps_info.latitude << ", " << gps_info.longitude << "\n";
    std::cout << E << ", " << N << "\n";
}

void GPS::convert_speed_course_to_velocity()
{
    double speed = gps_info.speed;
    double course = gps_info.course * MissionConstants::kDeg2Rad;

    gps_info.E_velocity = speed * std::sin(course);
    gps_info.N_velocity = speed * std::cos(course);
}

std::map<std::string, std::vector<std::string>> GPS::retrieve_all_NMEA_sentences()
{
    std::map<int, std::vector<std::string>> rmc_history;
    std::map<int, std::vector<std::string>> gga_history;

    std::map<std::string, std::vector<std::string>> history;

    std::vector<int> intersection_results;
    std::set<int> rmc_times;
    std::set<int> gga_times;

    for (auto &message : acculumated_messages)
    {
        std::vector<std::string> nmea_message_parts = break_message_down(message);
        std::string NMEA_type = determine_NMEA_type(nmea_message_parts);
        int time = stof(nmea_message_parts[MissionConstants::NMEA::TIME_IDX]);
        if (NMEA_type == MissionConstants::NMEA::RMC::RMC)
        {
            rmc_history[time] = nmea_message_parts;
            rmc_times.insert(time);
        }
        else if (NMEA_type == MissionConstants::NMEA::GGA::GGA)
        {
            gga_history[time] = nmea_message_parts;
            gga_times.insert(time);
        }
    }
    std::set_intersection(rmc_times.begin(), rmc_times.end(),
                          gga_times.begin(), gga_times.end(),
                          back_inserter(intersection_results));

    if (intersection_results.empty())
    {
        return history;
    }

    int max_common_element = intersection_results[intersection_results.size() - 1];

    history[MissionConstants::NMEA::RMC::RMC] = rmc_history[max_common_element];
    history[MissionConstants::NMEA::GGA::GGA] = gga_history[max_common_element];
    return history;
}

void GPS::read_data()
{
    if (fd < 0)
    {
        return;
    }

    auto now = std::chrono::system_clock::now();

    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();

    std::time_t tt = std::chrono::system_clock::to_time_t(seconds);

    std::tm tm;
    localtime_r(&tt, &tm); // thread-safe on Linux

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms;

    // It seems to only send 3 bytes at at time; Might have to do with frequency but probably not
    // Either way, this code fully reads the message and it will only break if buffer exceeds the max size of buffer
    int bytes_received = read(fd, buffer, sizeof(buffer));
    if (bytes_received <= 0)
    {
        return;
    }
    // std::cout << "Received " << bytes_received << "\n";
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
                // std::cout << message;
                acculumated_messages.push_back(message);
                this->GPSReceived << oss.str() << ", " << message << std::flush;
            }
            message.clear();
        }
    }

    memset(buffer, 0, static_cast<size_t>(bytes_received));
}

bool GPS::GPSAvailable()
{
    return valid;
}

void GPS::set_valid(const bool state)
{
    valid = state;
}
