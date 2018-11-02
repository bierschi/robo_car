//
// Created by christian on 03.09.18.
//

#include "server/ServerSocket.h"
#include "server/SocketException.h"

/**
 * Default Constructor for a ServerSocket instance
 *
 * USAGE:
 *      ServerSocket* sock = new ServerSocket();
 *
 */
ServerSocket::ServerSocket() : countSpeed(480),
                               distanceFlag(false),
                               forwardForbidden(false)
{

    steeringServo = new SteeringServo(15);
    cameraServo   = new CameraServo(14);
    ultrasonic    = new Ultrasonic(4, 5);
    gearmotor     = new GearMotor(26, 21);
    std::string mapname= "hector";
    //SlamMap sm(mapname, 65, 25);
    sm = new SlamMap(mapname, 65, 25);
    distance = ultrasonic->currentDistance();

    std::thread distanceThread(&ServerSocket::runDistanceThread, this);
    distanceThread.detach();

}

/**
 * Constructor for a ServerSocket instance
 *
 * USAGE:
 *      ServerSocket* sock = new ServerSocket(2500, 2);
 *
 * @param port: int
 */
ServerSocket::ServerSocket(unsigned int port, unsigned int maxClient) : port_n(port),
                                                                        maxClient_n(maxClient),
                                                                        countClient(0),
                                                                        connected(false){


    if ( !Socket::create() ) {

        throw SocketException("Could not create server socket!");

    }

    if ( !Socket::bind(port_n) ) {

        throw SocketException("Could not bind to port!");

    }

    if ( !Socket::listen() ) {

        throw SocketException("Could not listen to socket");

    }

    socks = new ServerSocket[maxClient_n];
    threadClients.reserve(maxClient_n);
    running = true;

    std::cout << "Server is being set up on port: " << port << std::endl;
    std::cout << maxClient_n << " clients can be connected simultaneously!" << std::endl;
    std::cout << "Listening ..." << std::endl;
}

/**
 * Destructor in ServerSocket
 */
ServerSocket::~ServerSocket() {

    running = false;
    delete[] socks;
    delete steeringServo, cameraServo, ultrasonic, gearmotor;
    threadClients.clear();

}

/**
 * get port of Server
 *
 * @return unsigned int port
 */
int ServerSocket::getPort() const {
    return port_n;
}

/**
 * returns true, if server is running
 *
 * @return bool running
 */
bool ServerSocket::isRunning() const {
    return running;
}

/**
 * get distanceFlag to ensure if distance thread is running
 *
 * @return bool distanceFlag
 */
bool ServerSocket::getDistanceFlag() {
    return distanceFlag;
}

/**
 * set distanceFlag to true or false
 * @param distFlag: bool distanceFlag
 */
void ServerSocket::setDistanceFlag(bool distFlag) {
    distanceFlag = distFlag;
}

/**
 * overloaded operator << to send strings to sockets
 *
 * @param s: const string reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::operator << (const std::string &s) const {

    if ( !Socket::send(s)) {

        throw SocketException("Could not write to socket!");

    }

    return *this;

}

/**
 * overloaded operator << to send predefined commands to sockets
 *
 * @param cmd: Commands& reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::operator << (Commands& cmd) const {

    if ( !Socket::send(cmd)) {

        throw SocketException("Could not write to socket!");

    }

    return *this;

}

/**
 * overloaded operator >> to receive strings from sockets
 *
 * @param s: string& reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::operator >> (std::string &s) const {

    if ( !Socket::recv(s)) {

        throw SocketException("Could not read from socket!");

    }

    return *this;
}

/**
 * overloaded operator >> to receive predefined commands from sockets
 *
 * @param cmd: Commands& reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::operator >> (Commands& cmd) const {

    if ( !Socket::recv(cmd) ) {

        throw SocketException("Could not read cmd from socket!");

    }

    return *this;
}

/**
 * blocking method, waits for a client connection
 *
 * @param sock: ServerSocket& reference
 */
void ServerSocket::accept(ServerSocket &sock) {

    if ( !Socket::accept(sock)) {

        throw SocketException("Could not accept socket!");

    }
}

/**
 * multiple clients can be connecting to server
 */
void ServerSocket::multipleClients() {
    std::clog << "Running Server with multiple client connections" << std::endl;

    std::thread runThread(&ServerSocket::processClient, this);
    runThread.join();
    //runThread.detach();
}

/**
 * run thread of server, which handles multiple client connections
 */
void ServerSocket::processClient() {


    while (isRunning()) {

        for (; countClient < maxClient_n; countClient++) {

            accept(socks[countClient]);
            std::cout << "countClient" << countClient << std::endl;
            threadClients.emplace_back(&ServerSocket::serveTask, this, std::ref(socks[countClient]));

            //threadClients[countClient].detach();
        }
        /*
        for (int j = 0; j < maxClient_n; j++) {
            std::clog << "join client into thread" << std::endl;

            threadClients[j].join();

        }*/
    }
}

/**
 * worker thread for connected clients. Send defined commands to server
 */
void ServerSocket::serveTask(ServerSocket& sock){

    try {

        Commands cmd;
        sock.connected = true;

        while (true) {

            sock >> cmd;
            sock.actions(cmd, sock);

        }

    } catch (SocketException& e) {

        std::cout << "Exception was caught in worker thread `serveTask`: " << e.description() << std::endl;
        //countClient--;
    }

}

/**
 * selects a appropriate action, depending on the incoming command
 *
 * @param cmd: Commands reference to execute defined commands
 * @param sock: ServerSocket reference to send commands back to the clients
 */
void ServerSocket::actions(Commands& cmd, ServerSocket& sock) {



    switch(cmd) {

        //move the vehicle
        case FORWARD:
            std::cout << "Drive Forward!" << std::endl;
            if ( !forwardForbidden) {
                countSpeed = 150;
                gearmotor->setSpeed(countSpeed);
            }

            break;

        case BACKWARD:
            std::cout << "Drive Backward!" << std::endl;

            countSpeed = -150;
            gearmotor->setSpeed(countSpeed);
            break;

        case STRAIGHT:
            std::cout << "Drive Straight Ahead!" << std::endl;

            steeringServo->driveStraight();
            break;

        case RIGHT:
            std::cout << "Drive Right!" << std::endl;

            steeringServo->driveRight();
            break;

        case LEFT:
            std::cout << "Drive Left!" << std::endl;

            steeringServo->driveLeft();
            break;

        case STOP:
            std::cout << "Stop vehicle!" << std::endl;

            gearmotor->stop();
            break;

        case INCREASE_SPEED:
            std::cout << "Increase current Speed!" << std::endl;

            if (gearmotor->getSpeed() < (gearmotor->getMaxSpeed() - 50)) {

                countSpeed += 50;
                gearmotor->setSpeed(countSpeed);

            }

            break;

        case DECREASE_SPEED:
            std::cout << "Decrease current Speed!" << std::endl;

            if (gearmotor->getSpeed() > (- gearmotor->getMaxSpeed() + 50)) {

                countSpeed -= 50;
                gearmotor->setSpeed(countSpeed);

            }
            break;

        //move the camera
        case CAM_R:
            std::cout << "Move camera right!" << std::endl;

            cameraServo->moveRight();
            break;

        case CAM_L:
            std::cout << "Move camera left!" << std::endl;

            cameraServo->moveLeft();
            break;

            //starts/stops the thread to get the current distance from ultrasonic sensor
        case DISTANCE: {
            std::cout << "Start/Stop to query current distance!" << std::endl;

            if ( !getDistanceFlag() ) {

                setDistanceFlag(true);
                //std::thread distanceThread(&ServerSocket::runDistanceThread, this, std::ref(sock));
                //distanceThread.detach();

            } else {

                setDistanceFlag(false);

            }

        }
            break;

        //starts the stream of the camera
        case STREAM: {

            std::cout << "Stream object!" << std::endl;
            //std::string mapname= "hector";
            //SlamMap sm(mapname, 65, 25);
            std::cout << "setsavemap" << std::endl;
            //sm->setSaveMap(true);

            //sock << "hallo";

        }

            break;

        default:
            std::cout << "Default in method actions!" << std::endl;
    }
}


/**
 * run thread to get continously the current distance from the ultrasonic sensor
 *
 * @param sock: ServerSocket reference to send the current distance to the clients
 */
void ServerSocket::runDistanceThread() {


    while (true) {

        distance = ultrasonic->currentDistance();

        if (distance < 12.0) {

            if ( (gearmotor->getSpeed() == 0) && (!forwardForbidden) ) {

                gearmotor->setSpeed(0);
                forwardForbidden = true;
            }

        }

        if (distance > 12.0){

            forwardForbidden = false;

        }
        //sleep(1);
        //std::cout << "distance: " << distance <<std::endl;
        usleep(400);
        //sock << std::to_string(distance);
    }

}