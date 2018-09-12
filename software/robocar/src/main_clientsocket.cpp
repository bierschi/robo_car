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

        ClientSocket client_socket ("localhost", 2501);

        Commands forward = FORWARD;
        Commands backward = BACKWARD;
        Commands right = RIGHT;
        Commands left = LEFT;
        Commands stream = STREAM;
        Commands cam_r = CAM_R;
        Commands cam_l = CAM_L;

        try {
            while (true) {

                std::string zeile;
                std::cout << "Message: ";
                std::getline(std::cin, zeile);

                if (zeile.compare(0, 7, "forward") == 0) {
                    client_socket << forward;
                } else if (zeile.compare(0, 8, "backward") == 0) {
                    client_socket << backward;
                } else if (zeile.compare(0, 5, "right") == 0) {
                    client_socket << right;
                } else if (zeile.compare(0, 4, "left") == 0) {
                    client_socket << left;
                }else if (zeile.compare(0, 6, "stream") == 0) {
                    client_socket << stream;
                }else if (zeile.compare(0, 5, "cam_r") == 0) {
                    client_socket << cam_r;
                }else if (zeile.compare(0, 5, "cam_l") == 0) {
                    client_socket << cam_l;
                }else {
                        exit(1);
                    }
                }

            } catch (SocketException& e) {

            std::cout<<"Inner Exception was caught: " << e.description() << std::endl;

            }

        } catch (SocketException& e) {

        std::cout << "Exception was caught: " << e.description() << std::endl;

        }

        return 0;
}
