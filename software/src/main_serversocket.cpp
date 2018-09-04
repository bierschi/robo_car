//
// Created by christian on 03.09.18.
//

#include <iostream>
#include <thread>
#include "cstring"
#include "cstdio"
#include "comm/ServerSocket.h"
#include "comm/SocketException.h"

#define MAX_CLIENTS 2

void serverTask(ServerSocket& socks){

    std::cout << "serverTask inside" << std::endl;

    try {

        while (true) {

            Commands cmd;

            socks >> cmd;
            socks.actions(cmd);

            }
        } catch ( ... ) {

            std::cout << "Ex: occured" << std::endl;
            
    }
}

int main() {
    std::cout << "Test ServerSocket" << std::endl;

    try {

        ServerSocket server (2501);
        ServerSocket* socks = new ServerSocket[MAX_CLIENTS];
        std::thread threads[MAX_CLIENTS];

        while (true) {

            for (int i =0; i<MAX_CLIENTS; i++) {

                server.accept(socks[i]);
                std::cout << "Accept client number: " << i+1 << std::endl;

                threads[i] = std::thread(serverTask, std::ref(socks[i]));
            }

            for (int j = 0; j < MAX_CLIENTS; j++) {

                std::cout << "join clients into thread" << std::endl;
                threads[j].join();

            }

        /*
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
        */
    }
}
catch ( SocketException& e) {

        std::cout << "Exception was caught: " << e.description() << std::endl;

    }
    return 0;
}
