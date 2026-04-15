// hardware/GPS.cpp

#include "GPS.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <set>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <exception>
#include "MissionConstants.hpp"
#include "Telemetry.hpp"

namespace
{
    bool TryParseIntToken(const std::vector<std::string> &parts, size_t index, int *out)
    {
        if (index >= parts.size() || parts[index].empty())
        {
            return false;
        }

        try
        {
            *out = std::stoi(parts[index]);
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    }
}

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
    update_count = 0;

    // Set Valid flag to false in constructor
    valid = false;
    fresh_position = false;
    fresh_velocity = false;
    has_last_rmc_time = false;
    has_last_gga_time = false;
    last_rmc_time = 0.0;
    last_gga_time = 0.0;

    fd = open(MissionConstants::GPS_Port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        Telemetry::GetInstance().Log("Warning: GPS port unavailable, continuing without GPS");
        found_gps = false;
    }
    else
    {
        found_gps = true;
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
        auto str = oss.str();

        GPSReceived.open("../logs/GPSReceived" + str + ".txt");

        // Get rid of any garbage values on startup
        tcflush(fd, TCIFLUSH);
    }
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

void GPS::Update()
{
    fresh_position = false;
    fresh_velocity = false;

    read_data();
    bool parsed_valid_sentence = false;

    for (const std::string &raw_message : acculumated_messages)
    {
        std::vector<std::string> nmea_message_parts = break_message_down(raw_message);
        if (nmea_message_parts.empty())
        {
            continue;
        }

        const std::string nmea_type = determine_NMEA_type(nmea_message_parts);
        if (nmea_type != MissionConstants::NMEA::RMC::RMC &&
            nmea_type != MissionConstants::NMEA::GGA::GGA)
        {
            continue;
        }

        if (parse_NMEA_type(nmea_type, nmea_message_parts))
        {
            parsed_valid_sentence = true;
        }
    }

    reset_acculumated_messages();

    set_valid(parsed_valid_sentence);
    if (!GPSAvailable())
    {
        return;
    }

    if (fresh_position)
    {
        convert_coordinate_frame();
    }

    if (fresh_position || fresh_velocity)
    {
        ++update_count;
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

bool GPS::parse_NMEA_type(const std::string nmea_message_type, const std::vector<std::string> &nmea_message_parts)
{
    if (nmea_message_type == MissionConstants::NMEA::RMC::RMC)
    {
        std::cout << "Got RMC output type " << "\n";
        if (nmea_message_parts.size() <= static_cast<size_t>(MissionConstants::NMEA::RMC::STATUS_IDX) ||
            nmea_message_parts[MissionConstants::NMEA::RMC::STATUS_IDX].empty())
        {
            std::cout << "RMC sentence missing status, skipping" << "\n";
            return false;
        }

        std::string status = nmea_message_parts[MissionConstants::NMEA::RMC::STATUS_IDX];
        char status_character = static_cast<char>(status[0]);
        if (status_character == MissionConstants::NMEA::RMC::BAD_STATUS_CHARACTER)
        {
            std::cout << "Data is not valid, failed to update GPS_info" << "\n";
            return false;
        }

        try
        {
            return parse_RMC(nmea_message_parts);
        }
        catch (const std::exception &e)
        {
            std::cout << "RMC parse failed: " << e.what() << "\n";
            return false;
        }
    }

    if (nmea_message_type == MissionConstants::NMEA::GGA::GGA)
    {
        std::cout << "Got GGA output type " << "\n";
        int status_int = 0;
        if (!TryParseIntToken(nmea_message_parts, MissionConstants::NMEA::GGA::STATUS_IDX, &status_int))
        {
            std::cout << "GGA sentence missing/invalid status, skipping" << "\n";
            return false;
        }

        if (status_int == MissionConstants::NMEA::GGA::BAD_STATUS_NUMBER)
        {
            std::cout << "Data is not valid, failed to update GPS_info" << "\n";
            return false;
        }

        try
        {
            return parse_GGA(nmea_message_parts);
        }
        catch (const std::exception &e)
        {
            std::cout << "GGA parse failed: " << e.what() << "\n";
            return false;
        }
    }

    return false;
}

bool GPS::parse_RMC(const std::vector<std::string> &nmea_message_parts)
{
    if (nmea_message_parts.size() <= static_cast<size_t>(MissionConstants::NMEA::RMC::COURSE_IDX) ||
        nmea_message_parts.size() <= static_cast<size_t>(MissionConstants::NMEA::RMC::SPEED_IDX) ||
        nmea_message_parts.size() <= static_cast<size_t>(MissionConstants::NMEA::RMC::LONGITUDE_DIRECTION_IDX) ||
        nmea_message_parts[MissionConstants::NMEA::TIME_IDX].empty() ||
        nmea_message_parts[MissionConstants::NMEA::RMC::LATITUDE_IDX].empty() ||
        nmea_message_parts[MissionConstants::NMEA::RMC::LONGITUDE_IDX].empty() ||
        nmea_message_parts[MissionConstants::NMEA::RMC::COURSE_IDX].empty() ||
        nmea_message_parts[MissionConstants::NMEA::RMC::SPEED_IDX].empty() ||
        nmea_message_parts[MissionConstants::NMEA::RMC::LATITUDE_DIRECTION_IDX].empty() ||
        nmea_message_parts[MissionConstants::NMEA::RMC::LONGITUDE_DIRECTION_IDX].empty())
    {
        throw std::invalid_argument("RMC sentence missing required fields");
    }

    const double time = std::stod(nmea_message_parts[MissionConstants::NMEA::TIME_IDX]);
    if (has_last_rmc_time && std::abs(time - last_rmc_time) < 1e-6)
    {
        return false;
    }
    has_last_rmc_time = true;
    last_rmc_time = time;

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
    fresh_position = true;
    fresh_velocity = true;
    return true;
}

bool GPS::parse_GGA(const std::vector<std::string> &nmea_message_parts)
{
    if (nmea_message_parts.size() <= static_cast<size_t>(MissionConstants::NMEA::GGA::ALTITUDE_INDEX) ||
        nmea_message_parts[MissionConstants::NMEA::TIME_IDX].empty() ||
        nmea_message_parts[MissionConstants::NMEA::GGA::ALTITUDE_INDEX].empty())
    {
        throw std::invalid_argument("GGA sentence missing required fields");
    }

    const double time = std::stod(nmea_message_parts[MissionConstants::NMEA::TIME_IDX]);
    if (has_last_gga_time && std::abs(time - last_gga_time) < 1e-6)
    {
        return false;
    }
    has_last_gga_time = true;
    last_gga_time = time;

    float altitude = stof(nmea_message_parts[MissionConstants::NMEA::GGA::ALTITUDE_INDEX]);

    // Sanity check:
    std::cout << "Time: " << time << "\n";
    std::cout << "Altitude: " << altitude << "\n";
    std::cout << "\n";

    gps_info.altitude = altitude;
    fresh_position = true;
    return true;
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
}

void GPS::convert_speed_course_to_velocity()
{
    double speed = gps_info.speed;
    double course = gps_info.course * MissionConstants::kDeg2Rad;

    gps_info.E_velocity = speed * std::sin(course);
    gps_info.N_velocity = speed * std::cos(course);
}

void GPS::read_data()
{
    // Safety check
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

    // Either way, this code fully reads the message and it will only break if buffer exceeds the max size of buffer
    // which might happeb if the gps settings were not set so we are reading every sentence each 10 times per second
    int bytes_received = read(fd, buffer, sizeof(buffer));

    // Safety check
    if (bytes_received <= 0)
    {
        return;
    }

    for (int i = 0; i < bytes_received; ++i)
    {
        char character = buffer[i];
        message += character;

        // If we are at the end of a NMEA message, let's first make sure that we recieved a full message by trying to find '$'
        if (character == '\n')
        {
            if (message.find('$') != std::string::npos)
            {
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

bool GPS::GPSUsed()
{
    return found_gps;
}

bool GPS::HasFreshPosition() const
{
    return fresh_position;
}

bool GPS::HasFreshVelocity() const
{
    return fresh_velocity;
}

uint64_t GPS::GetUpdateCount() const
{
    return update_count;
}

void GPS::set_valid(const bool state)
{
    valid = state;
}