//
// Created by christian on 03.09.18.
//

#ifndef ROBOCAR_SERVERSOCKET_H
#define ROBOCAR_SERVERSOCKET_H

#include <thread>
#include <vector>

#include "Socket.h"
#include "sensors/PCA9685.h"
#include "sensors/Ultrasonic.h"

/**
 * /CLASS ServerSocket
 *
 * creates a ServerSocket object
 */
class ServerSocket : private Socket {

private:
    unsigned int port_n;
    unsigned int maxClient_n;
    int countClient;
    int countCamServo;
    bool running = false;
    bool distanceFlag = false;

    std::vector<std::thread> threadClients;

    ServerSocket* socks;
    PCA9685* steeringServo;
    PCA9685* cameraServo;
    Ultrasonic* ultrasonic;

public:

    ServerSocket();
    ServerSocket(unsigned int port, unsigned int maxClient_n = 2);
    virtual ~ServerSocket();

    const ServerSocket& operator << (const std::string& ) const;
    const ServerSocket& operator << (Commands& ) const;
    const ServerSocket& operator >> (std::string& ) const;
    const ServerSocket& operator >> (Commands& ) const;

    int getPort() const;
    bool getDistanceFlag();
    void setDistanceFlag(bool distFlag);

    void accept(ServerSocket&);
    bool isRunning() const;

    void processClient();
    void serveTask(ServerSocket&);
    void multipleClients();

    void actions(Commands&, ServerSocket& sock);
    void continousMeasurement(ServerSocket& sock);
};

#endif //ROBOCAR_SERVERSOCKET_H
