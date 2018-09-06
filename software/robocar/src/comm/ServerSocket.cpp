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
ServerSocket::ServerSocket(int port) {

    if ( !Socket::create() ) {

        throw SocketException("Could not create server socket!");

    }

    if ( !Socket::bind(port) ) {

        throw SocketException("Could not bind to port!");

    }

    if ( !Socket::listen() ) {

        throw SocketException("Could not listen to socket");

    }
}

ServerSocket::~ServerSocket() {

}

const ServerSocket& ServerSocket::operator << (const std::string &s) const {

    if ( !Socket::send(s)) {

        throw SocketException("Could not write to socket!");

    }

    return *this;

}

const ServerSocket& ServerSocket::operator >> (std::string &s) const {

    if ( !Socket::recv(s)) {

        throw SocketException("Could not read from socket!");

    }

    return *this;
}

const ServerSocket& ServerSocket::operator >> (Commands& cmd) const {

    if ( !Socket::recv(cmd) ) {

        throw SocketException("Could not read cmd from socket!");

    }

    return *this;
}

void ServerSocket::accept(ServerSocket &sock) {

    if ( !Socket::accept(sock)) {

        throw SocketException("Could not accept socket!");

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
            break;

        case BACKWARD:
            std::cout << "Drive Backward!" << std::endl;
            break;

        case RIGHT:
            std::cout << "Drive Right!" << std::endl;
            break;

        case LEFT:
            std::cout << "Drive Left!" << std::endl;
            break;

        case STREAM:
            std::cout << "Stream object!" << std::endl;
            break;

        default:
            std::cout << "Default in act!" << std::endl;
    }
}