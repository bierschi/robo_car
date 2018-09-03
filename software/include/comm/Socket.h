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
    bool bind(const int port);
    bool listen() const;
    bool accept(Socket&) const;

    bool connect(const std::string host, const int port);

    bool send(const std::string) const;
    int recv(std::string&) const;

    bool isValid() const { return m_sock != -1;}
    void setNonBlocking(const bool);

};

#endif //ROBOCAR_SOCKET_H
