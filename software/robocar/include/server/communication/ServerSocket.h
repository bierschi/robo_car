//
// Created by christian on 03.09.18.
//

#ifndef ROBOCAR_SERVERSOCKET_H
#define ROBOCAR_SERVERSOCKET_H

#include <thread>
#include <vector>
#include <iomanip>
#include <unistd.h>

#include "Socket.h"

/**
 * /CLASS ServerSocket
 *
 * creates a ServerSocket object
 */
class ServerSocket : private Socket {

private:
    unsigned int port_;

public:

    ServerSocket();
    ServerSocket(unsigned int port);
    virtual ~ServerSocket();

    const ServerSocket& sending(const std::string& s) const;
    const ServerSocket& sending(Commands& cmd) const;
    const ServerSocket& sending(std::vector<int>& v);
    const ServerSocket& receiving (std::string& ) const;
    const ServerSocket& receiving (Commands& ) const;

    int getPort() const;

    void accept(ServerSocket&);

    //void processClient();
    //void serveTask(ServerSocket&);
    //void multipleClients();

};

#endif //ROBOCAR_SERVERSOCKET_H
