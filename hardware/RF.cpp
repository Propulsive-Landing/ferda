#include <string>
#include <cstring>
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
#include <termios.h>

#include "MissionConstants.hpp"
#include "RF.hpp"
#include "Telemetry.hpp"

RF::RF()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto str = oss.str();

    RFSent.open("../logs/RFSent" + str + ".txt");

    // OPEN SERIAL PORT FOR HARDWARE
    SerialFd = open(MissionConstants::RF_Port, O_RDWR);
    // SerialPort = fopen("./virtual_rf.txt", "w+");

    // If RF through XBEE fails, switch to terminal
    if (SerialFd < 0)
    {
        // TODO: FIGURE OUT HOW TO INDI RECURSIVE DEPENEDNECY AND USE TELEMTRY LOGGING HERE
        // Telemetry::GetInstance().Log("Switching to terminal controls");
        std::cout << "Switching to terminal controls" << "\n";
        terminal_switch = true;
    }
    else
    {
        int flags = fcntl(SerialFd, F_GETFL, 0);
        fcntl(SerialFd, F_SETFL, flags | O_NONBLOCK);
    }
}

RF::~RF()
{
    RFSent.close();

    if (!terminal_switch)
        // CLOSE SERIAL PORT
        close(SerialFd);
}

void RF::SendString(std::string text)
{
    // Add time tag to file
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    if (!terminal_switch)
    {
        write(SerialFd, text.c_str(), sizeof(char) * text.size());
    }

    // write time to file
    this->RFSent << std::put_time(std::localtime(&in_time_t), "%c") << ",";
    this->RFSent << text << "\n"
                 << std::flush;
}

RF::Command RF::GetCommand() // Will check for commands and return the received command. Non-blocking.
{

    // struct pollfd fds;
    // int ret;
    // fds.fd = SerialFd; /* this is Serial Port */
    // fds.events = POLLIN;
    // ret = poll(&fds, 1, 0);

    // std::cout << "Polling " << std::to_string(ret) << "\n";

    // if(ret != 1) // Return if no data
    //     return RF::Command::None;

    std::string input_line;
    if (!terminal_switch)
    {
        const int MAXLEN = 512;
        char buffer[MAXLEN];
        memset(buffer, 0, 512);
        int len = read(SerialFd, buffer, MAXLEN);

        if (len <= 0)
        {
            return RF::Command::None;
        }

        std::cout << "GOT: " << buffer;

        input_line = buffer;

        std::cout << "String:" << input_line << "\n"
                  << std::flush;

        for (size_t i = 0; i < 100; ++i)
        {
            std::cout << static_cast<int>(buffer[i]) << " "; // Output the byte values as integers
        }
        std::cout << std::endl;
    }
    else
    {
        struct pollfd fds;
        int ret;
        fds.fd = 0; /* this is STDIN */
        fds.events = POLLIN;
        ret = poll(&fds, 1, 0);

        if (ret != 1) // Return if no data
            return RF::Command::None;

        // Extra safety check before reading
        if (std::cin.eof() || !std::cin.good())
            return RF::Command::None;

        std::getline(std::cin, input_line);

        std::cout << "GOT: " << input_line << "\n"
                  << std::flush;
    }

    return ParseCommand(input_line);
}