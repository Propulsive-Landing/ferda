#include <string>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <iostream>
#include <poll.h>

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
}

RF::~RF()
{
    RFSent.close();
}

void RF::SendString(std::string message)
{
    RFSent << message << "\n";
}

RF::Command RF::GetCommand() // Will check for commands and return the received command. Non-blocking. Called frequently
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

    std::string input_line;
    std::getline(std::cin, input_line);

    std::cout << "GOT: " << input_line << "\n"
              << std::flush;

    return ParseCommand(input_line);
}