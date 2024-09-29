#pragma once

/**
 * @file UDPClient.hpp
 * @brief Singleton UDP client for simulation communication
 *
 * This class handles bidirectional UDP communication for a simulation environment.
 * It sends actuator commands and receives sensor data using separate threads.
 *
 * Usage:
 *   UDPClient& client = UDPClient::GetInstance();
 *   auto [x, y, z] = client.GetBodyAngularRate();
 *   client.SetXServo(45.0);
 *
 * Public methods:
 * - GetBodyAngularRate(): Returns tuple of angular rates (x, y, z)
 * - GetBodyAcceleration(): Returns tuple of accelerations (x, y, z)
 * - GetPressure(): Returns current pressure
 * - GetTemperature(): Returns current temperature
 * - SetXServo(angle): Sets X servo angle
 * - SetYServo(angle): Sets Y servo angle
 * - Ignite(is_launch): Triggers ignition (true for launch, false for landing)
 *
 * Implementation details:
 * - Uses separate read and write threads for async communication
 * - Ensures thread safety with mutex for shared data access
 * - Binds to a local port for receiving, sends to predefined server address/port
 *
 * Potential modifications:
 * - Update ReadThread() and WriteThread() to change data format
 * - Add new member variables and methods for additional sensors/actuators
 *
 * Note: Update this documentation when modifying the UDPClient class.
 */

#include <iostream>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include <tuple>

#ifdef _WIN32
    #include <winsock2.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <unistd.h>
    #include <arpa/inet.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
#endif

class UDPClient {
private:
    UDPClient() : running(false), socket_fd(-1) {
        Initialize();
    }
    ~UDPClient() { Cleanup(); }

    int socket_fd;
    struct sockaddr_in server_addr;
    std::thread read_thread;
    std::thread write_thread;
    std::atomic<bool> running;

    std::mutex data_mutex;

    // Shared variables for sensor data
    std::array<double, 3> angular_rate{0};
    std::array<double, 3> acceleration{0};
    double temperature{0};
    double pressure{0};

    // Shared variables for actuator data sending

    bool should_send = false;

    double motor_1_ignition{0};
    double motor_2_ignition{0};
    double motor_angle_x{0};
    double motor_angle_y{0};

    void ReadThread() {
        char buffer[64];
        while (running) {
            int recv_len = recvfrom(socket_fd, buffer, sizeof(buffer), 0, nullptr, nullptr);
            if (recv_len == sizeof(double) * 8) {
                std::lock_guard<std::mutex> lock(data_mutex);
                memcpy(angular_rate.data(), buffer, sizeof(double) * 3);
                memcpy(acceleration.data(), buffer + sizeof(double) * 3, sizeof(double) * 3);
                memcpy(&temperature, buffer + sizeof(double) * 6, sizeof(double));
                memcpy(&pressure, buffer + sizeof(double) * 7, sizeof(double));
            }
        }
    }

    void WriteThread() {
        char buffer[32];
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Adjust as needed
            
            if (should_send) {
                std::lock_guard<std::mutex> lock(data_mutex);

                memcpy(buffer, &motor_1_ignition, sizeof(double));
                memcpy(buffer + sizeof(double), &motor_2_ignition, sizeof(double));
                memcpy(buffer + sizeof(double) * 2, &motor_angle_x, sizeof(double));
                memcpy(buffer + sizeof(double) * 3, &motor_angle_y, sizeof(double));
            }
            if (should_send) {
                sendto(socket_fd, buffer, sizeof(buffer), 0, 
                       (struct sockaddr*)&server_addr, sizeof(server_addr));
                should_send = false;
            }
        }
    }

    bool Initialize() {
        #ifdef _WIN32
        WSADATA wsa_data;
        if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
            std::cerr << "Failed to initialize Winsock" << std::endl;
            return false;
        }
        #endif

        socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd == -1) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }

        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(SIM_LOCAL_PORT);

        if (bind(socket_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) == -1) {
            std::cerr << "Bind failed" << std::endl;
            return false;
        }

        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr(SIM_SERVER_IP);
        server_addr.sin_port = htons(SIM_SERVER_PORT);

        running = true;
        read_thread = std::thread(&UDPClient::ReadThread, this);
        write_thread = std::thread(&UDPClient::WriteThread, this);

        return true;
    }

public:
    static UDPClient& GetInstance() {
        static UDPClient instance;
        return instance;
    }

    void Cleanup() {
        if (running) {
            running = false;
            if (read_thread.joinable()) read_thread.join();
            if (write_thread.joinable()) write_thread.join();

            #ifdef _WIN32
            closesocket(socket_fd);
            WSACleanup();
            #else
            close(socket_fd);
            #endif
        }
    }

    std::tuple<double, double, double> GetBodyAngularRate() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return std::make_tuple(angular_rate[0], angular_rate[1], angular_rate[2]);
    }

    std::tuple<double, double, double> GetBodyAcceleration() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return std::make_tuple(acceleration[0], acceleration[1], acceleration[2]);
    }

    double GetPressure() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return pressure;
    }

    double GetTemperature() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return temperature;
    }

    void SetXServo(double angle) {
        std::lock_guard<std::mutex> lock(data_mutex);
        motor_angle_x = angle;
        should_send = true;
    }

    void SetYServo(double angle) {
        std::lock_guard<std::mutex> lock(data_mutex);
        motor_angle_y = angle;
        should_send = true;
    }

    void Ignite(bool is_launch) {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (is_launch) {
            motor_1_ignition = 1.0;
        } else {
            motor_2_ignition = 1.0;
        }
        should_send = true;
    }

    UDPClient(const UDPClient&) = delete;
    UDPClient& operator=(const UDPClient&) = delete;
};