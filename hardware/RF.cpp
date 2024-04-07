#include <string>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <sstream>
#include <chrono>

#include "RF.hpp"

RF::RF()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto str = oss.str();

    RFSent.open ("../logs/RFSent"+str+".txt");

    // OPEN SERIAL PORT FOR HARDWARE
    SerialPort = fopen("/dev/ttyS0", "rw");
    if (SerialPort < 0)
        throw std::runtime_error("failed to open serial port");
}

RF::~RF()
{
    RFSent.close();

    // CLOSE SERIAL PORT
    fclose(SerialPort);
}



void RF::SendString(std::string text)
{
    // Add time tag to file
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);


    const char * str = text.c_str();

    unsigned char length;
    for(length = 0; length <= 254 && str[length] != '\0'; length++){} //Gets size of str without requirement for strnlen include


    unsigned int packet_size = sizeof(uint32_t) + sizeof(uint8_t) + length + sizeof(uint32_t); //packet size (derived from packet structure above)

    uint8_t packet[packet_size];

    uint32_t header = STRING_MAGIC_NUMBER;
    uint32_t ending = RF_FOOTER;

    uint8_t * ending_ptr = (uint8_t *) &ending;
    uint8_t * header_ptr = (uint8_t *) &header;

    unsigned int packet_index = 0;

    //add header to packet
    for(unsigned int i = 0; i < sizeof(uint32_t); i++)
    {
        packet[packet_index] = header_ptr[i];
        packet_index++;
    }

    //add size byte to packet
    packet[packet_index] = length;
    packet_index++;

    //add string to packet
    for(unsigned int i = 0; i < length; i++)
    {
        packet[packet_index] = str[i];
        packet_index++;
    }

    //add footer to packet
    for(unsigned int i = 0; i < sizeof(uint32_t); i++)
    {
        packet[packet_index] = ending_ptr[i];
        packet_index++;
    }

    std::string result((char *) packet, packet_size);

    fwrite(result.c_str(), sizeof(char), result.size(), SerialPort);
    fflush(SerialPort);

    // write time to file
    this->RFSent << std::put_time(std::localtime(&in_time_t), "%c") << ",";
    this->RFSent << text << "\n" << std::flush;
}