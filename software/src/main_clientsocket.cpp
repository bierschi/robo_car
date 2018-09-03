//
// Created by christian on 03.09.18.
//

#include <iostream>
#include "cstring"
#include "cstdio"
#include "comm/ClientSocket.h"
#include "comm/SocketException.h"

int main() {
    std::cout << "Test ClientSocket" << std::endl;
    try {


        ClientSocket client_socket ("localhost", 2500);

        std::string reply;

        try {
            while (true) {

                std::string zeile;
                std::cout << "Message: ";
                std::getline(std::cin, zeile);
                client_socket << zeile;

            }


        } catch (SocketException&) {}

        }
        catch (SocketException& e) {

        std::cout << "Exception was caught: " << e.description() << "\n";
    }

        return 0;
}
