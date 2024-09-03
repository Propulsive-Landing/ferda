#include <string>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <iostream>
#include <poll.h>

#include "RF.hpp"

RF::RF()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto str = oss.str();

    RFSent.open ("../logs/RFSent"+str+".txt");
}

RF::~RF()
{
    RFSent.close();
}



void RF::SendString(std::string message)
{
    RFSent << message << "\n";
}

void RF::SendFrame(RF::rfFrame frame)
{
    
    frame.magic_number = FRAME_MAGIC_NUMBER;
    frame.footer = RF_FOOTER;

    uint8_t * packet = (uint8_t *) &frame;

    std::string result((char *) packet, sizeof(RF::rfFrame));

    SendString(result);
}

RF::Command RF::GetCommand() // Will check for commands and return the received command. Non-blocking.
{
    struct pollfd fds;
    int ret;
    fds.fd = 0; /* this is STDIN */
    fds.events = POLLIN;
    ret = poll(&fds, 1, 0);

    if(ret != 1) // Return if no data
        return RF::Command::None;

    std::string input_line;
    std::getline(std::cin, input_line);
    std::cout << "GOT: " << input_line << "\n" << std::flush;

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