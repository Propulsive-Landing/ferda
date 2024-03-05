#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 
#include <arpa/inet.h>
#include <netdb.h>
#include <iostream>
#include <chrono>
#include <vector>

#include "SimulationManager.hpp"

Eigen::Matrix<double, 7, 1> ParseOutput(std::string s){
    Eigen::Matrix<double, 7, 1> output;

    std::string delimiter = ",";

    unsigned int index = 0;
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);

        output(index, 0) = stod(token);
        // std::cout << token << std::endl;

        s.erase(0, pos + delimiter.length());
        index += 1;
    }

    return output;
}



Eigen::Matrix<double, 7, 1> SimulationManager::GetOutputs()
{
    static bool firstRun = true;
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    if( firstRun || ( seconds > POLL_TIME && validConnection)){
        firstRun = false;
        start_time = std::chrono::high_resolution_clock::now();
        // Update stored data

        // sending data 
        std::ostringstream ss;
        ss << SimInputs(0, 0) << ",";
        ss << SimInputs(1, 0) << ",";
        ss << SimInputs(2, 0) << ",";
        ss << SimInputs(3, 0);

        std::string s = ss.str();

        const char* message = s.c_str();
        
        int result = send(clientSocket, message, strlen(message), 0); 

        if(result == -1){
            validConnection = false;
            return SimulationOutputs;
        }

        char buf[512];
        result = recv(clientSocket, buf, 512, 0);

        if(result == -1){
            validConnection = false;
            return SimulationOutputs;
        }

        Eigen::Matrix<double, 7, 1> parsed = ParseOutput(buf);
        
        SimulationOutputs = parsed;

    }

    return SimulationOutputs;
}



void SimulationManager::SetInputs(Eigen::Matrix<double, 4, 1> inputs)
{
    SimInputs = inputs;
}


SimulationManager::~SimulationManager()
{
    close(clientSocket);
}

SimulationManager::SimulationManager()
{
    // creating socket 
    clientSocket = socket(AF_INET, SOCK_STREAM, 0); 
  
    // Find the ip
    struct hostent *he=gethostbyname(hostname);
    char *ip=inet_ntoa(*(struct in_addr*)he->h_addr_list[0]);

    // specifying address 
    sockaddr_in serverAddress; 
    serverAddress.sin_family = AF_INET; 
    serverAddress.sin_port = htons(port); 
    serverAddress.sin_addr.s_addr = inet_addr(ip);

    
  
    // sending connection request 
    int result = connect(clientSocket, (struct sockaddr*)&serverAddress, 
            sizeof(serverAddress));  

    if(result == -1){
        validConnection = false;
        std::cout << "FAILED TO CONNECT TO SIMULATION SERVER";
    }
    else{
        validConnection = true;
        std::cout << "Successfully connected to simulation server";
    }
}
