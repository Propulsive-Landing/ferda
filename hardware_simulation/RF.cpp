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
    RF::Command ParsedCommand = RF::Command::None;

    if(input_line == "ABORT")
        ParsedCommand = RF::Command::ABORT;
    else if(input_line == "Startup")
        ParsedCommand = RF::Command::Startup;
    else if(input_line == "TestTVC")
        ParsedCommand = RF::Command::TestTVC;
    else if(input_line == "GoIdle")
        ParsedCommand = RF::Command::GoIdle;
    else if(input_line == "Ignite")
        ParsedCommand = RF::Command::Ignite;
    else if(input_line == "IncrementYTVC")
        ParsedCommand = RF::Command::IncrementYTVC;
    else if(input_line == "IncrementXTVC")
        ParsedCommand = RF::Command::IncrementXTVC;
    else if(input_line == "DecrementXTVC")
        ParsedCommand = RF::Command::DecrementXTVC;
    else if(input_line == "DecrementYTVC")
        ParsedCommand = RF::Command::DecrementYTVC;
    else if(input_line == "Release")
        ParsedCommand = RF::Command::Release;
    else
        ParsedCommand = RF::Command::None;

    return ParsedCommand;
}