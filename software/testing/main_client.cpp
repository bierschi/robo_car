//
// Created by christian on 28.08.18.
//

#include <iostream>
#include "cstring"
#include "cstdio"
#include "client.h"


int main() {
    std::cout << "Test Client Class" << std::endl;
    std::string host = "localhost";
    unsigned int port = 2500;
    Client client(host, port);
    client.run();

    while(true){
        client.send();
    }

    return 0;
}

