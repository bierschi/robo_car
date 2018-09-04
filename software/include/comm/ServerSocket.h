//
// Created by christian on 03.09.18.
//

#ifndef ROBOCAR_SERVERSOCKET_H
#define ROBOCAR_SERVERSOCKET_H

#include "Socket.h"

/**
 * /CLASS ServerSocket
 *
 * creates a ServerSocket object
 */
class ServerSocket : private Socket {

public:

    ServerSocket(int port);
    ServerSocket(){};
    virtual ~ServerSocket();

    const ServerSocket& operator << (const std::string& ) const;
    const ServerSocket& operator >> (std::string& ) const;
    const ServerSocket& operator >> (Commands& ) const;

    void accept(ServerSocket&);

    void actions(const std::string&);
    void actions(Commands&);
};

#endif //ROBOCAR_SERVERSOCKET_H
