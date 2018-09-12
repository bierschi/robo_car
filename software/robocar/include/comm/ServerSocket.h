//
// Created by christian on 03.09.18.
//

#ifndef ROBOCAR_SERVERSOCKET_H
#define ROBOCAR_SERVERSOCKET_H

#include <thread>
#include <vector>

#include "Socket.h"
#include "sensors/PCA9685.h"

/**
 * /CLASS ServerSocket
 *
 * creates a ServerSocket object
 */
class ServerSocket : private Socket {

private:
    unsigned int port_n;
    unsigned int maxClient_n;
    ServerSocket* socks;
    std::vector<std::thread> threadClients;
    bool running = false;
    int countClient;
    int camCounter;

public:

    ServerSocket(){};
    ServerSocket(unsigned int port, unsigned int maxClient_n = 2);
    virtual ~ServerSocket();

    const ServerSocket& operator << (const std::string& ) const;
    const ServerSocket& operator >> (std::string& ) const;
    const ServerSocket& operator >> (Commands& ) const;

    int getPort() const;
    void accept(ServerSocket&);
    bool isRunning() const;

    void processClient();
    void serveTask(ServerSocket&);
    void multipleClients();

    void actions(const std::string&);
    void actions(Commands&);
};

#endif //ROBOCAR_SERVERSOCKET_H
