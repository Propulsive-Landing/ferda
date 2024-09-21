#include <string>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <iostream>
#include <poll.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>


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
    SerialFd = open("/dev/ttyS0", O_RDWR | O_NONBLOCK);
    // SerialPort = fopen("./virtual_rf.txt", "w+");
    if (SerialFd < 0)
        throw std::runtime_error("failed to open serial port");
}

RF::~RF()
{
    RFSent.close();

    // CLOSE SERIAL PORT
    close(SerialFd);
}

void RF::SendFrame(RF::rfFrame frame)
{
    
    frame.magic_number = FRAME_MAGIC_NUMBER;
    frame.footer = RF_FOOTER;

    uint8_t * packet = (uint8_t *) &frame;

    std::string result((char *) packet, sizeof(RF::rfFrame));

    write(SerialFd, result.c_str(), sizeof(char)*result.size());
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

    // fwrite(result.c_str(), sizeof(char), result.size(), SerialPort);
    // fflush(SerialPort);
    
    write(SerialFd, result.c_str(), sizeof(char)*result.size());

    // write time to file
    this->RFSent << std::put_time(std::localtime(&in_time_t), "%c") << ",";
    this->RFSent << text << "\n" << std::flush;
}


RF::Command RF::GetCommand() // Will check for commands and return the received command. Non-blocking.
{

    struct pollfd fds;
    int ret;
    fds.fd = SerialFd; /* this is Serial Port */
    fds.events = POLLIN;
    ret = poll(&fds, 1, 0);

    
    std::cout << "Polling " << std::to_string(ret) << "\n";

    if(ret != 1) // Return if no data
        return RF::Command::None;

    const int MAXLEN = 512;
    char buffer[MAXLEN];
    int len = read(SerialFd, buffer, MAXLEN);

    std::cout << "GOT: " << buffer << "\n" << std::flush;

    std::string input_line(buffer);    

    // if statement for which command to return
    if(input_line == "ABORT")
        return RF::Command::ABORT;
    else if(input_line == "Startup")
        return RF::Command::Startup;
    else if(input_line == "Ignite")
        return RF::Command::Ignite;
    else if(input_line == "Release")
        return RF::Command::Release;
    else
        return RF::Command::None;
}