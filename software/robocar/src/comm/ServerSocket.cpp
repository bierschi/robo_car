//
// Created by christian on 03.09.18.
//

#include "comm/ServerSocket.h"
#include "comm/SocketException.h"

/**
 * Default Constructor for a ServerSocket instance
 *
 * USAGE:
 *      ServerSocket* sock = new ServerSocket();
 *
 */
ServerSocket::ServerSocket() : countCamServo(2120){

    steeringServo = new PCA9685(1, 0x40, 60);
    cameraServo   = new PCA9685(1, 0x40, 60);
    ultrasonic    = new Ultrasonic(4, 5);

}

/**
 * Constructor for a ServerSocket instance
 *
 * USAGE:
 *      ServerSocket* sock = new ServerSocket(2500, 2);
 *
 * @param port: int
 */
ServerSocket::ServerSocket(unsigned int port, unsigned int maxClient) : port_n(port), maxClient_n(maxClient), countClient(0){


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
    delete steeringServo, cameraServo;
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
}

/**
 * run thread of server, which handles multiple client connections
 */
void ServerSocket::processClient() {


    while (isRunning()) {

        for (; countClient < maxClient_n; countClient++) {

            accept(socks[countClient]);
            threadClients.emplace_back(&ServerSocket::serveTask, this, std::ref(socks[countClient]));

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

        while (true) {

            sock >> cmd;
            sock.actions(cmd, sock);

        }

    } catch (SocketException& e) {

        std::cout << "Exception was caught in worker thread `serveTask`: " << e.description() << std::endl;
        countClient--;
    }

}

/**
 * selects a appropriate action, depending on the incoming command
 *
 * @param cmd: Commands&
 */
void ServerSocket::actions(Commands& cmd, ServerSocket& sock) {

    switch(cmd) {

        //move the vehicle
        case FORWARD:
            std::cout << "Drive Forward!" << std::endl;
            steeringServo->setPWM(0, 1750, 2130);
            //cameraServo.setPWM(15, 1750, 2128);
            break;

        case BACKWARD:
            std::cout << "Drive Backward!" << std::endl;
            break;

        case RIGHT:
            std::cout << "Drive Right!" << std::endl;
            steeringServo->setPWM(0, 1230, 1720);
            //cameraServo.setPWM(15, 1780, 2000);
            break;

        case LEFT:
            std::cout << "Drive Left!" << std::endl;
            steeringServo->setPWM(0, 1750, 1895);
            //cameraServo.setPWM(15, 1230, 1750);
            break;

        //move the camera
        case CAM_R:
            std::cout << "Move camera right!" << std::endl;

            if (countCamServo > 1820){

                countCamServo -= 30;
                cameraServo->setPWM(15, 1750, countCamServo);

            }
            break;

        case CAM_L:
            std::cout << "Move camera left!" << std::endl;

            if (countCamServo < 2320){

            countCamServo += 30;
            cameraServo->setPWM(15, 1750, countCamServo);

            }
            break;

            //starts the stream of the camera
        case DISTANCE: {
            std::cout << "Get current distance!" << std::endl;
            double distance = ultrasonic->triggerOneMeasurement();
            sock << std::to_string(distance);
        }
            break;

        //starts the stream of the camera
        case STREAM:
            std::cout << "Stream object!" << std::endl;
            sock << "hallo";
            break;

        default:
            std::cout << "Default in method actions!" << std::endl;
    }
}
