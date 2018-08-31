//
// Created by christian on 28.08.18.
//

#include <iostream>
#include "cstring"
#include "cstdio"
#include "comm/client.h"

int main() {
    std::cout << "Test Client Class" << std::endl;
    std::string host = "localhost";
    unsigned int port = 2503;
    Client client(host, port);
    client.run();
    client.send();
    client.recv();
    client.send();
    client.recv();

    return 0;
}

