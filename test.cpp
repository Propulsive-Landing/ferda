// C++ program to illustrate the client application in the 
// socket programming 
#include <cstring> 
#include <iostream> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 
#include <arpa/inet.h>
#include <netdb.h>
  
int main() 
{ 
    // creating socket 
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0); 
  
    // Find the ip
    struct hostent *he=gethostbyname("6.tcp.ngrok.io");
    char *ip=inet_ntoa(*(struct in_addr*)he->h_addr_list[0]);

    // specifying address 
    sockaddr_in serverAddress; 
    serverAddress.sin_family = AF_INET; 
    serverAddress.sin_port = htons(14475); 
    serverAddress.sin_addr.s_addr = inet_addr(ip);

    
  
    // sending connection request 
    connect(clientSocket, (struct sockaddr*)&serverAddress, 
            sizeof(serverAddress)); 
  
    // sending data 
    const char* message = "10,2,3,22"; 
    send(clientSocket, message, strlen(message), 0); 
  
    char buf[512];
    recv(clientSocket, buf, 512, 0);

    std::cout << buf << "\n";


    // closing socket 
    close(clientSocket); 
    
  
    return 0; 
}