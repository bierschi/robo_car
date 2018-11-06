//
// Created by christian on 03.09.18.
//

#include "server/communication/ServerSocket.h"
#include "server/communication/SocketException.h"

/**
 * Default Constructor for a ServerSocket instance
 *
 * USAGE:
 *      ServerSocket* sock = new ServerSocket();
 *
 */
ServerSocket::ServerSocket()
{


}

/**
 * Constructor for a ServerSocket instance
 *
 * USAGE:
 *      ServerSocket* sock = new ServerSocket(2500, 2);
 *
 * @param port: int
 */
ServerSocket::ServerSocket(unsigned int port) : port_(port),
                                                connected_(false){


    if ( !Socket::create() ) {

        throw SocketException("Could not create server socket!");

    }

    if ( !Socket::bind(port_) ) {

        throw SocketException("Could not bind to port!");

    }

    if ( !Socket::listen() ) {

        throw SocketException("Could not listen to socket");

    }

    running_ = true;

}

/**
 * Destructor in ServerSocket
 */
ServerSocket::~ServerSocket() {

    running_ = false;
    connected_ = false;

    delete sock;

}

/**
 * get port of Server
 *
 * @return unsigned int port
 */
int ServerSocket::getPort() const {
    return port_;
}

/**
 * returns true, if server is running
 *
 * @return bool running
 */
bool ServerSocket::getRunningFlag() const {
    return running_;
}


/**
 * overloaded operator << to send strings to sockets
 *
 * @param s: const string reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::sending (const std::string &s) const {

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
const ServerSocket& ServerSocket::sending (Commands& cmd) const {

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
const ServerSocket& ServerSocket::receiving(std::string &s) const {

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
const ServerSocket& ServerSocket::receiving (Commands& cmd) const {

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
 *
 */
/*
void ServerSocket::multipleClients() {
    std::clog << "Running Server with multiple client connections" << std::endl;

    std::thread runThread(&ServerSocket::processClient, this);
    runThread.join();
    //runThread.detach();
}
*/

/**
 * run thread of server, which handles multiple client connections
 */
/*
void ServerSocket::processClient() {


    while (isRunning()) {

        for (; countClient < maxClient_n; countClient++) {

            accept(socks[countClient]);
            std::cout << "countClient" << countClient << std::endl;
            threadClients.emplace_back(&ServerSocket::serveTask, this, std::ref(socks[countClient]));

            //threadClients[countClient].detach();
        }

    }
}
*/

/**
 * worker thread for connected clients. Send defined commands to server
 */
 /*
void ServerSocket::serveTask(ServerSocket& sock){

    try {

        Commands cmd;
        sock.connected_ = true;

        while (true) {

            sock.receiving(cmd);
            //sock.actions(cmd, sock);

        }

    } catch (SocketException& e) {

        std::cout << "Exception was caught in worker thread `serveTask`: " << e.description() << std::endl;
        //countClient--;
    }

}

*/
/**
 * selects a appropriate action, depending on the incoming command
 *
 * @param cmd: Commands reference to execute defined commands
 * @param sock: ServerSocket reference to send commands back to the clients
 */
/*
void ServerSocket::actions(Commands& cmd) {

    switch(cmd) {

        //move the vehicle
        case FORWARD:
            std::cout << "Drive Forward!" << std::endl;

            break;

        case BACKWARD:
            std::cout << "Drive Backward!" << std::endl;

            break;

        case STRAIGHT:
            std::cout << "Drive Straight Ahead!" << std::endl;

            break;

        case RIGHT:
            std::cout << "Drive Right!" << std::endl;

            break;

        case LEFT:
            std::cout << "Drive Left!" << std::endl;

            break;

        case STOP:
            std::cout << "Stop vehicle!" << std::endl;

            break;

        case INCREASE_SPEED:
            std::cout << "Increase current Speed!" << std::endl;

            break;

        case DECREASE_SPEED:
            std::cout << "Decrease current Speed!" << std::endl;

            break;

        //move the camera
        case CAM_R:
            std::cout << "Move camera right!" << std::endl;
            break;

        case CAM_L:
            std::cout << "Move camera left!" << std::endl;
            break;

            //starts/stops the thread to get the current distance from ultrasonic sensor
        case DISTANCE: {
            std::cout << "Start/Stop to query current distance!" << std::endl;
        }
            break;

        //starts the stream of the camera
        case STREAM: {
            std::cout << "Stream object!" << std::endl;
        }

            break;

        default:
            std::cout << "Default in method actions!" << std::endl;
    }
}

*/