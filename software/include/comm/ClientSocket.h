//
// Created by christian on 03.09.18.
//

#ifndef ROBOCAR_CLIENTSOCKET_H
#define ROBOCAR_CLIENTSOCKET_H

#include "comm/Socket.h"

/**
 * /CLASS ClientSocket
 *
 * creates a ClientSocket object
 */
class ClientSocket : private Socket {

public:

    ClientSocket(std::string host, int port);
    virtual ~ClientSocket(){};

    const ClientSocket& operator << (const std::string&) const;
    const ClientSocket& operator >> (std::string&) const;

};

#endif //ROBOCAR_CLIENTSOCKET_H
