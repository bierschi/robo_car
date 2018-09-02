//
// Created by Christian on 27.08.2018.
//

#ifndef ROBOCAR_CLIENT_H
#define ROBOCAR_CLIENT_H

#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

#define BUFFSIZE 1000

/**
 * /CLASS Client
 *
 * creates a client object
 */
class Client {

private:
    std::string& host;
    unsigned int& port;

    int sockfd, n;
    char buf[BUFFSIZE];
    struct sockaddr_in serv_addr;
    struct hostent* server;

    bool connected;

public:
    Client(std::string host, unsigned int port);
    ~Client();

    std::string getHost();
    unsigned int getPort();

    void stop();
    void run();
    void send();
    void recv();

};
#endif //ROBOCAR_CLIENT_H
