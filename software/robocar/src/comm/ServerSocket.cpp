//
// Created by christian on 03.09.18.
//

#include "comm/ServerSocket.h"
#include "comm/SocketException.h"


/**
 * Constructor for a ServerSocket instance
 *
 * USAGE:
 *      ServerSocket(2500);
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

    steeringServo = new PCA9685(1, 0x40);
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
 *
 * @param s
 * @return const ServerSocket&
 */
const ServerSocket& ServerSocket::operator << (const std::string &s) const {

    if ( !Socket::send(s)) {

        throw SocketException("Could not write to socket!");

    }

    return *this;

}

/**
 *
 * @param s
 * @return const ServerSocket&
 */
const ServerSocket& ServerSocket::operator >> (std::string &s) const {

    if ( !Socket::recv(s)) {

        throw SocketException("Could not read from socket!");

    }

    return *this;
}

/**
 *
 * @param cmd
 * @return const ServerSocket&
 */
const ServerSocket& ServerSocket::operator >> (Commands& cmd) const {

    if ( !Socket::recv(cmd) ) {

        throw SocketException("Could not read cmd from socket!");

    }

    return *this;
}

/**
 * blocking method, waits for a client connection
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
            sock.actions(cmd);

        }

    } catch (SocketException& e) {

        std::cout << "Exception was caught in worker thread `serveTask`: " << e.description() << std::endl;
        countClient--;
    }

}

/**
 * selects a appropriate action, depending on the incoming data string
 *
 * @param data: std::string&
 */
void ServerSocket::actions(const std::string &data) {

    if (data.compare(0, 4, "stop") == 0) {

        std::cout << "stop running Server!" << std::endl;

    }
    else if (data.compare(0, 4, "send") == 0){

        std::cout << "send message to client!" << std::endl;


    } else if (data.compare(0, 6, "stream") == 0) {

        std::cout << "create stream object!" << std::endl;


    } else if (data.compare(0, 7, "forward") == 0) {

        std::cout << "drive forward!" << std::endl;


    } else if (data.compare(0, 8, "backward") == 0) {

        std::cout << "drive backward!" << std::endl;


    } else if (data.compare(0, 5, "right") == 0) {

        std::cout << "drive right!" << std::endl;


    } else if (data.compare(0, 4, "left") == 0) {

        std::cout << "drive left!" << std::endl;

    }
    else {
        std::cout << "get next message from client" << std::endl;
        std::string nextMsg = "get next message from client";

    }

}

void ServerSocket::actions(Commands& cmd) {

    switch(cmd) {
        case FORWARD:
            std::cout << "Drive Forward!" << std::endl;
            steeringServo->setPWM(1, 1750, 2130);
            break;

        case BACKWARD:
            std::cout << "Drive Backward!" << std::endl;
            break;

        case RIGHT:
            std::cout << "Drive Right!" << std::endl;
            steeringServo.setPWM(1, 1230, 1750);
            break;

        case LEFT:
            std::cout << "Drive Left!" << std::endl;
            steeringServo.setPWM(1, 1750, 1895);
            break;

        case STREAM:
            std::cout << "Stream object!" << std::endl;
            break;

        default:
            std::cout << "Default in act!" << std::endl;
    }
}