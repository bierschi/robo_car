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

/** /class Server
 *
 *  creates a Server object
 */
class Server {

private:
    unsigned int& port;

    int sockfd, newsockfd;
    socklen_t clientLen;
    char buffer[BUFFER_LEN];
    struct sockaddr_in serverAddr, clientAddr;
    long dataLen;

    bool running;

public:
    Server(unsigned int port_n);
    ~Server();

    bool isRunning();
    int getPort();

    void run();
    void recv();
    void send(const std::string&);
    void stop();
    void actions(const std::string&);

};


#endif //ROBOCAR_SERVER_H
