//
// Created by christian on 03.09.18.
//

#include <iostream>
#include "cstring"
#include "cstdio"
#include "comm/ServerSocket.h"
#include "comm/SocketException.h"

int main() {
    std::cout << "Test ServerSocket" << std::endl;

    try {

    ServerSocket server (2500);

    while (true) {

        ServerSocket new_sock;
        server.accept(new_sock);

        try {
            while (true) {
                Commands cmd;

                std::cout <<"read: " << std::endl;

                new_sock >> cmd;
                new_sock.actions(cmd);
                std::cout << "Server received: " << cmd << std::endl;

            }
        }
        catch ( SocketException& e){

            std::cout << "Inner Exception was caught: " << e.description() << std::endl;

        }
    }
}
catch ( SocketException& e) {

        std::cout << "Exception was caught: " << e.description() << std::endl;

    }
    return 0;
}
