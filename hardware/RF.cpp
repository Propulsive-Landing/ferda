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
        // Non-blocking
        int flags = fcntl(SerialFd, F_GETFL, 0);
        fcntl(SerialFd, F_SETFL, flags | O_NONBLOCK);

        struct termios tty;
        tcgetattr(SerialFd, &tty);

        // Set baud rate
        cfsetispeed(&tty, B38400);
        cfsetospeed(&tty, B38400);

        // 8N1
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag |= (CLOCAL | CREAD);

        // Disable echo + canonical mode
        tty.c_lflag &= ~(ECHO | ICANON | ECHOE | ISIG);

        // Disable flow control
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // disable CR/LF translation
        tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);

        // Raw output
        tty.c_oflag &= ~OPOST;

        // Apply
        tcsetattr(SerialFd, TCSANOW, &tty);
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
        const char *data = text.c_str();
        size_t totalBytes = text.size();
        size_t bytesWritten = 0;

        while (bytesWritten < totalBytes)
        {
            ssize_t bytes = write(
                SerialFd,
                data + bytesWritten,
                totalBytes - bytesWritten);

            if (bytes > 0)
            {
                bytesWritten += static_cast<size_t>(bytes);
            }
            else if (bytes == -1)
            {
                if (errno == EINTR)
                {
                    std::cout << "Write interrupted, retrying..." << std::endl;
                    continue; // interrupted, retry
                }
                else if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    std::cout << "Write would block, retrying..." << std::endl;
                    continue; // would block, retry (or add sleep if needed)
                }
                else
                {
                    perror("write failed");
                    break; // or throw if you prefer
                }
            }
        }
    }

    // write time to file
    this->RFSent << std::put_time(std::localtime(&in_time_t), "%c") << ",";
    this->RFSent << text << "\n"
                 << std::flush;
}

RF::Command RF::GetCommand() // Will check for commands and return the received command. Non-blocking.
{
    if (!terminal_switch)
    {
        RF::Command cmd = RF::Command::None;
        static std::string rx_buffer;

        char buffer[512];
        int len = read(SerialFd, buffer, sizeof(buffer));

        if (len > 0)
        {
            rx_buffer.append(buffer, len);

            size_t pos;
            while ((pos = rx_buffer.find('\n')) != std::string::npos)
            {
                std::string line = rx_buffer.substr(0, pos);
                rx_buffer.erase(0, pos + 1);

                std::cout << "FULL MSG: " << line << "\n";

                cmd = ParseCommand(line); // process ALL
            }
        }

        return cmd;
    }
    else
    {
        std::string input_line;
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

        return ParseCommand(input_line);
    }
}
