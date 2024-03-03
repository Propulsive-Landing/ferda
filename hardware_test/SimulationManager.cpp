#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 
#include <arpa/inet.h>
#include <netdb.h>
#include <iostream>
#include <chrono>

#include "SimulationManager.hpp"


Eigen::Matrix<double, 7, 1> GetOutputs()
{
    static auto start_time = std::chrono::high_resolution_clock::now();
    int milliseconds_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    double seconds = milliseconds_since_start / 1000.0;

    if( seconds > POLL_TIME && validConnection){
        start_time = std::chrono::high_resolution_clock::now();
        // Update stored data

        // sending data 
        std::ostringstream ss;
        ss << SimInputs(0, 0) << ",";
        ss << SimInputs(1, 0) << ",";
        ss << SimInputs(2, 0) << ",";
        ss << SimInputs(3, 0);

        const char* message = ss.str().c_str()
        
        send(clientSocket, message, strlen(message), 0); 

    }

    return SimulationOuptuts;
}



void SetInputs(Eigen::Matrix<double, 4, 1> inputs)
{
    SimInputs = inputs;
}


SimulationManager::SimulationManager()
{
    // creating socket 
    clientSocket = socket(AF_INET, SOCK_STREAM, 0); 
  
    // Find the ip
    struct hostent *he=gethostbyname("6.tcp.ngrok.io");
    char *ip=inet_ntoa(*(struct in_addr*)he->h_addr_list[0]);

    // specifying address 
    sockaddr_in serverAddress; 
    serverAddress.sin_family = AF_INET; 
    serverAddress.sin_port = htons(14475); 
    serverAddress.sin_addr.s_addr = inet_addr(ip);

    
  
    // sending connection request 
    int result = connect(clientSocket, (struct sockaddr*)&serverAddress, 
            sizeof(serverAddress));  

    if(result == -1){
        validConnection = false
        std::cout << "FAILED TO CONNECT TO SIMULATION SERVER";
    }
    else{
        validConnection = true
        std::cout << "Successfully connected to simulation server";
    }
}
