//
// Created by christian on 03.09.18.
//

#include "communication/ClientSocket.h"
#include "communication/SocketException.h"

/**
 * Constructor for a ClientSocket instance
 *
 * USAGE:
 *      ClientSocket("localhost", 2500);
 *
 * @param host: std::string
 * @param port: int port
 */
ClientSocket::ClientSocket(std::string host, int port) {

    if ( !Socket::create() ) {

        throw SocketException("Could not create client socket!");

    }

    if ( !Socket::connect(host, port)) {

        throw SocketException("Could not bind to port!");

    }
}

/**
 * method to send strings to sockets
 *
 * @param s: const string reference
 * @return const ClientSocket& reference
 */
const ClientSocket& ClientSocket::sending (const std::string &s) const {

    if ( !Socket::send(s)) {

        throw SocketException("Could not write to socket!");

    }

    return *this;
}

/**
 * method to send predefined commands to sockets
 *
 * @param cmd: Commands& reference
 * @return const ClientSocket& reference
 */
const ClientSocket& ClientSocket::sending (Commands& cmd) const {

    if ( !Socket::send(cmd)) {

        throw SocketException("Could not write ctl cmd to socket!");

    }
}

/**
 * method to receive strings from sockets
 *
 * @param s: string& reference
 * @return const ClientSocket& reference
 */
const ClientSocket& ClientSocket::receiving (std::string &s) const {

    if ( !Socket::recv(s)) {

        throw SocketException("Could not read from socket!");

    }

    return *this;
}

/**
 * method to receive predefined commands from sockets
 *
 * @param cmd: Commands& reference
 * @return const ClientSocket& reference
 */
const ClientSocket& ClientSocket::receiving (Commands& cmd) const {

    if ( !Socket::recv(cmd)) {

        throw SocketException("Could not read ctl cmd from socket!");

    }

    return *this;
}

/**
 * disconnect cleanly from the socket
 */
void ClientSocket::closeConnection() {

    Socket::disconnect();

}