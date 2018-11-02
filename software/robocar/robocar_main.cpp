//
// Created by christian on 03.09.18.
//

#include <iostream>

#include "Car.h"
#include "ros/ros.h"

#define LOGGING true


int main(int argc, char** argv) {
    std::cout << "Test Robotic Car" << std::endl;

    Car* car = new Car();

    car->driveForward(50);
    sleep(1);
    car->driveStop();
    sleep(1);
    car->driveBackward(100);
    sleep(1);
    car->driveStop();

    /*

    if (!LOGGING) {
        std::clog.setstate(std::ios_base::failbit);
    }

    try {

        ros::init(argc, argv, "robocar");
        ServerSocket server (2501, 1);
        server.multipleClients();


        while(ros::ok()) {


            ros::spinOnce();

        }

    }
    catch ( SocketException& e) {

        std::cout << "Exception was caught: " << e.description() << std::endl;

    }
    */
    return 0;

}
