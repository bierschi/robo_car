//
// Created by Christian on 27.08.2018.
//

#ifndef ROBOCAR_CLIENT_H
#define ROBOCAR_CLIENT_H
#include <string>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define BUFFSIZE 1000

class Client {

private:
    std::string host;
    unsigned int port;
    int sockfd;
    bool connected;
    char recv[BUFFSIZE];
    struct sockaddr_in serverAddr;

public:
    Client(std::string &host, unsigned int &port);
    ~Client();

    void connect();
    void send();
    std::string read();



};
#endif //ROBOCAR_CLIENT_H
