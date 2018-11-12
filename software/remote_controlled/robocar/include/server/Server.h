//
// Created by christian on 02.11.18.
//

#ifndef ROBOCAR_SERVER_H
#define ROBOCAR_SERVER_H

#include "server/communication/ServerSocket.h"
#include "server/communication/SocketException.h"
#include "car/Car.h"

#include "std_msgs/String.h"


/**
 * /CLASS Server
 *
 * creates a Server object
 */
class Server {

private:
    unsigned int port_;
    ServerSocket* serverSocket;
    ServerSocket* sock, *mapSocket;
    Commands cmd;

    Car& car_;

public:
    Server(unsigned int port, Car& car);
    ~Server();

    void startThread();
    void waitForClient();
    void run();
    void actions(Commands& cmd);
    void sendDataAtStart();

};

#endif //ROBOCAR_SERVER_H
