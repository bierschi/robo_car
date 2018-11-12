//
// Created by christian on 03.09.18.
//

#ifndef ROBOCAR_CLIENTSOCKET_H
#define ROBOCAR_CLIENTSOCKET_H

#include "Socket.h"

/**
 * /CLASS ClientSocket
 *
 * creates a ClientSocket object
 */
class ClientSocket : private Socket {

public:

    ClientSocket(std::string host, int port);
    virtual ~ClientSocket(){};

    const ClientSocket& sending (const std::string&) const;
    const ClientSocket& sending (Commands&) const;

    const ClientSocket& receiving (std::string&) const;
    const ClientSocket& receiving (Commands&) const;

    void closeConnection();
};

#endif //ROBOCAR_CLIENTSOCKET_H
