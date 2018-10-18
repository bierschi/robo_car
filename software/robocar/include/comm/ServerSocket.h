//
// Created by christian on 03.09.18.
//

#ifndef ROBOCAR_SERVERSOCKET_H
#define ROBOCAR_SERVERSOCKET_H

#include <thread>
#include <vector>
#include <iomanip>

#include "Socket.h"
#include "sensors/Ultrasonic.h"
#include "sensors/GearMotor.h"
#include "sensors/SteeringServo.h"
#include "sensors/CameraServo.h"

/**
 * /CLASS ServerSocket
 *
 * creates a ServerSocket object
 */
class ServerSocket : private Socket {

private:
    unsigned int port_n;
    unsigned int maxClient_n;
    int countClient, countSpeed;
    double distance;
    bool running = false;
    bool distanceFlag = false;

    std::vector<std::thread> threadClients;

    ServerSocket* socks;
    SteeringServo* steeringServo;
    CameraServo* cameraServo;
    Ultrasonic* ultrasonic;
    GearMotor* gearmotor;

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
    void runDistanceThread(ServerSocket& sock);
};

#endif //ROBOCAR_SERVERSOCKET_H
