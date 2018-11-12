//
// Created by christian on 04.11.18.
//
#include <iostream>
#include "ClientSocket.h"

int main(int argc, char** argv) {
    std::cout << "Test Client" << std::endl;
    std::string host = "localhost";
    unsigned int port = 2501;
    ClientSocket* cs = new ClientSocket(host, port);

    while (true) {
        (*cs) << "abc";
        sleep(1);
    }

    return 0;

}
