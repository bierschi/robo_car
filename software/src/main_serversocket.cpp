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
                std::cout <<"read: " << std::endl;
                std::string data;
                new_sock >> data;
                new_sock.actions(data);
                //std::cout << "Server received: " << data << std::endl;
            }
        }
        catch ( SocketException&){}
    }
}
catch ( SocketException& e) {
        std::cout << "Exception was caught: " << e.description() << "\n";
    }
    return 0;
}
