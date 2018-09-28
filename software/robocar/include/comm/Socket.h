//
// Created by christian on 03.09.18.
//

#ifndef ROBOCAR_SOCKET_H
#define ROBOCAR_SOCKET_H

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string>

const int MAXCONNECTIONS = 5;
const int MAXRECV = 500;

enum Commands {FORWARD, BACKWARD, STRAIGHT, RIGHT, LEFT, STOP, INCREASE_SPEED, DECREASE_SPEED, CAM_R, CAM_L, STREAM, DISTANCE};

/** /CLASS Socket
 *
 * defines the basic functions for a server client connection
 *
 */
class Socket {

private:

    int m_sock;
    sockaddr_in m_addr;

public:

    Socket();
    virtual ~Socket();

    bool create();
    bool bind(const unsigned int port);
    bool listen() const;
    bool accept(Socket&) const;

    bool connect(const std::string host, const int port);

    bool send(const std::string) const;
    bool send(Commands&) const;

    int recv(std::string&) const;
    int recv(Commands&) const;

    bool isValid() const { return m_sock != -1;}
    void setNonBlocking(const bool);

};

#endif //ROBOCAR_SOCKET_H
