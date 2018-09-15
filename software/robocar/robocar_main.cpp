//
// Created by christian on 03.09.18.
//

#include <iostream>
#include <thread>
#include "cstring"
#include "cstdio"
#include "comm/ServerSocket.h"
#include "comm/SocketException.h"

#define LOGGING true


int main() {
    std::cout << "Test ServerSocket" << std::endl;

    if (!LOGGING) {
        std::clog.setstate(std::ios_base::failbit);
    }

    try {

        ServerSocket server (2501, 2);
        server.multipleClients();

    }
    catch ( SocketException& e) {

        std::cout << "Exception was caught: " << e.description() << std::endl;

    }

    return 0;

}
