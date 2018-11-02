//
// Created by christian on 03.09.18.
//

#include <iostream>
#include <thread>
#include "cstring"
#include "cstdio"
#include "comm/ServerSocket.h"
#include "comm/SocketException.h"

#include "ros/ros.h"

#define LOGGING true


int main(int argc, char** argv) {
    std::cout << "ServerSocket" << std::endl;

    if (!LOGGING) {
        std::clog.setstate(std::ios_base::failbit);
    }

    try {

        ros::init(argc, argv, "robocar");
        ServerSocket server (2501, 1);
        server.multipleClients();

        /*
        while(ros::ok()) {


            ros::spinOnce();

        }*/

    }
    catch ( SocketException& e) {

        std::cout << "Exception was caught: " << e.description() << std::endl;

    }

    return 0;

}
