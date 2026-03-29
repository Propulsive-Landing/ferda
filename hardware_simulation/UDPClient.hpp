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
#include <chrono>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include <tuple>
#include <cstdint>
#include <limits>
#include <cmath>
#include <iomanip>
#include <Eigen/Dense>

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
    UDPClient() : socket_fd(-1), running(false) {
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
    std::array<double, 3> acceleration{0};
    std::array<double, 3> angular_rate{0};
    std::array<double, 3> magnetic_field{0};
    std::array<double, 3> gps_position{0};
    std::array<double, 2> gps_velocity{0};
    std::array<double, 9> camera_vectors{0};
    std::array<double, 1> lidar_data{0};
    double camera_frame_id{-1.0};
    double simulation_time_k{0.0};

    // Shared variables for actuator data sending

    bool should_send = false;

    double motor_angle_x{0};
    double motor_angle_y{0};
    double motor_1_ignition{0};
    double thrust{0};
    std::array<double, 3> nav_position_e{0};
    std::array<double, 3> nav_velocity_e{0};

    void ReadThread() {
        constexpr int kRxDoubles = 26; // [t_k, accel(3), gyro(3), mag(3), gps_pos(3), gps_vel(2), camera(9), frame_ID(1), lidar(1)]
        constexpr int kRxBytes = static_cast<int>(sizeof(double) * kRxDoubles);
        constexpr int kStatsPrintEvery = 200;

        char buffer[256];
        std::uint64_t packet_count = 0;
        std::uint64_t bad_size_count = 0;
        std::uint64_t duplicate_or_reordered_count = 0;
        std::uint64_t estimated_dropped_count = 0;
        double last_t_k = std::numeric_limits<double>::quiet_NaN();
        double dt_avg = 0.0;

        while (running) {
            int recv_len = recvfrom(socket_fd, buffer, sizeof(buffer), 0, nullptr, nullptr);

            if (recv_len != kRxBytes) {
                ++bad_size_count;
                if (bad_size_count <= 5 || (bad_size_count % 100) == 0) {
                    std::cout << "UDP RX warning: expected " << kRxBytes
                              << " bytes, got " << recv_len
                              << " (bad_size_count=" << bad_size_count << ")" << std::endl;
                }
                continue;
            }

            double t_k = 0.0;
            memcpy(&t_k, buffer, sizeof(double));

            ++packet_count;
            if (!std::isnan(last_t_k)) {
                const double dt_k = t_k - last_t_k;
                if (dt_k <= 0.0) {
                    ++duplicate_or_reordered_count;
                } else {
                    if (dt_avg <= 0.0) {
                        dt_avg = dt_k;
                    } else {
                        dt_avg = 0.98 * dt_avg + 0.02 * dt_k;
                    }

                    if (dt_avg > 0.0) {
                        const int inferred_steps = static_cast<int>(std::llround(dt_k / dt_avg));
                        if (inferred_steps > 1) {
                            estimated_dropped_count += static_cast<std::uint64_t>(inferred_steps - 1);
                        }
                    }
                }
            }
            last_t_k = t_k;

            {
                std::lock_guard<std::mutex> lock(data_mutex);
                simulation_time_k = t_k;
                memcpy(acceleration.data(), buffer + sizeof(double) * 1, sizeof(double) * 3);
                memcpy(angular_rate.data(), buffer + sizeof(double) * 4, sizeof(double) * 3);
                memcpy(magnetic_field.data(), buffer + sizeof(double) * 7, sizeof(double) * 3);
                memcpy(gps_position.data(), buffer + sizeof(double) * 10, sizeof(double) * 3);
                memcpy(gps_velocity.data(), buffer + sizeof(double) * 13, sizeof(double) * 2);
                memcpy(camera_vectors.data(), buffer + sizeof(double) * 15, sizeof(double) * 9);
                memcpy(&camera_frame_id, buffer + sizeof(double) * 24, sizeof(double));
                memcpy(lidar_data.data(), buffer + sizeof(double) * 25, sizeof(double));
            }

            
            if ((packet_count % kStatsPrintEvery) == 0) {
                const std::uint64_t total_with_est_drops = packet_count + estimated_dropped_count;
                const double drop_pct = (total_with_est_drops > 0)
                    ? (100.0 * static_cast<double>(estimated_dropped_count) / static_cast<double>(total_with_est_drops))
                    : 0.0;
                std::cout << std::fixed << std::setprecision(6)
                          << "UDP RX stats: packets=" << packet_count
                          << ", est_dropped=" << estimated_dropped_count
                          << " (" << std::setprecision(2) << drop_pct << "%)"
                          << std::setprecision(6)
                          << ", dup_or_reordered=" << duplicate_or_reordered_count
                          << ", bad_size=" << bad_size_count
                          << ", last_t_k=" << t_k
                          << ", dt_avg=" << dt_avg
                          << std::endl;
            }
            
        }
    }

    void WriteThread() {
        constexpr int kTxDoubles = 11; // [t_k, tvc_x, tvc_y, ignition, thrust, nav_x, nav_y, nav_z, nav_vx, nav_vy, nav_vz]
        char buffer[sizeof(double) * kTxDoubles];
        while (running) {
           std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Adjust as needed
            
            if (should_send) {
                std::lock_guard<std::mutex> lock(data_mutex);

                memcpy(buffer, &simulation_time_k, sizeof(double));
                memcpy(buffer + sizeof(double), &motor_angle_x, sizeof(double));
                memcpy(buffer + sizeof(double) * 2, &motor_angle_y, sizeof(double));
                memcpy(buffer + sizeof(double) * 3, &motor_1_ignition, sizeof(double));
                memcpy(buffer + sizeof(double) * 4, &thrust, sizeof(double));
                memcpy(buffer + sizeof(double) * 5, nav_position_e.data(), sizeof(double) * 3);
                memcpy(buffer + sizeof(double) * 8, nav_velocity_e.data(), sizeof(double) * 3);
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
 
        std::cout << "Attempting local bind on port " << std::to_string(SIM_LOCAL_PORT) << std::endl;

        if (bind(socket_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) == -1) {
            std::cerr << "Bind failed on port " << std::to_string(SIM_LOCAL_PORT) << std::endl;

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

    std::tuple<double, double, double> GetMagneticField() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return std::make_tuple(magnetic_field[0], magnetic_field[1], magnetic_field[2]);
    }

    std::tuple<double, double, double> GetGPSPosition() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return std::make_tuple(gps_position[0], gps_position[1], gps_position[2]);
    }

    std::tuple<double, double> GetGPSVelocity() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return std::make_tuple(gps_velocity[0], gps_velocity[1]);
    }

    std::tuple<double> GetLidarDistance() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return std::make_tuple(lidar_data[0]);
    }

    std::tuple<double, double, double, double, double, double, double, double, double> GetUnitVectors() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return std::make_tuple(
            camera_vectors[0], camera_vectors[1], camera_vectors[2],
            camera_vectors[3], camera_vectors[4], camera_vectors[5],
            camera_vectors[6], camera_vectors[7], camera_vectors[8]
        );
    }

    double GetCameraFrameId() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return camera_frame_id;
    }

    void SetTVCX(double angle) {
        std::lock_guard<std::mutex> lock(data_mutex);
        motor_angle_x = angle;
        should_send = true;
    }

    void SetTVCY(double angle) {
        std::lock_guard<std::mutex> lock(data_mutex);
        motor_angle_y = angle;
        should_send = true;
    }

    void SetThrust(double thrust_N) {
        std::lock_guard<std::mutex> lock(data_mutex);
        thrust = thrust_N;
        should_send = true;
    }

    void SetNavigationState(const Eigen::Vector3d &position_e, const Eigen::Vector3d &velocity_e) {
        std::lock_guard<std::mutex> lock(data_mutex);
        nav_position_e[0] = position_e(0);
        nav_position_e[1] = position_e(1);
        nav_position_e[2] = position_e(2);
        nav_velocity_e[0] = velocity_e(0);
        nav_velocity_e[1] = velocity_e(1);
        nav_velocity_e[2] = velocity_e(2);
        should_send = true;
    }

    void Ignite(bool is_launch) {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (is_launch) {
            motor_1_ignition = 1.0;
        }
        should_send = true;
    }

    UDPClient(const UDPClient&) = delete;
    UDPClient& operator=(const UDPClient&) = delete;
};
