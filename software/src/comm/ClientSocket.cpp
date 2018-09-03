//
// Created by christian on 03.09.18.
//

#include "comm/ClientSocket.h"
#include "comm/SocketException.h"

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

const ClientSocket& ClientSocket::operator<<(const std::string &s) const {

    if ( !Socket::send(s)) {

        throw SocketException("Could not write to socket!");

    }

    return *this;
}

const ClientSocket& ClientSocket::operator>>(std::string &s) const {

    if ( !Socket::recv(s)) {

        throw SocketException("Could not read from socket!");

    }

    return *this;
}
