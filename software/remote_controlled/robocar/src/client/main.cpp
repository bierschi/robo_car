//
// Created by christian on 04.11.18.
//
#include <iostream>
#include "client/ClientSocket.h"

int main(int argc, char** argv) {

    std::cout << "Test Client" << std::endl;

    std::string host = "localhost";
    unsigned int port = 2501;
    ClientSocket* client = new ClientSocket(host, port);
    Commands forward = FORWARD;
    Commands backward = BACKWARD;

    while (true) {
        client->sending(forward);
        sleep(2);
        client->sending(backward);
    }

    return 0;

}
