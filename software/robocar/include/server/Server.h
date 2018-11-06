//
// Created by christian on 02.11.18.
//

#ifndef ROBOCAR_SERVER_H
#define ROBOCAR_SERVER_H

#include "server/communication/ServerSocket.h"
#include "server/communication/SocketException.h"
#include "car/Car.h"


/**
 * /CLASS Server
 *
 * creates a Server object
 */
class Server {

private:
    unsigned int port_;
    ServerSocket* serverSocket;
    ServerSocket* sock;
    Commands cmd;

    Car& car_;

public:
    Server(unsigned int port, Car& car);
    ~Server();

    void waitForClient();
    void run();
    void actions(Commands& cmd);


};

#endif //ROBOCAR_SERVER_H
