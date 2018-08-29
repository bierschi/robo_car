//
// Created by christian on 28.08.18.
//

#ifndef ROBOCAR_SERVER_H
#define ROBOCAR_SERVER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>

#define BUFFER_LEN 256

class Server {

private:
    int sockfd, newsockfd, port;
    socklen_t clilen;
    char buffer[BUFFER_LEN];
    bool running;
    struct sockaddr_in serv_addr, cli_addr;
    long dataLen;

public:
    Server(int port_n);
    ~Server();

    void run();
    void recv();
    void send(const std::string&);
    bool isRunning();
    int getPort();
    void stop();
    void actions(const std::string&);

};


#endif //ROBOCAR_SERVER_H
