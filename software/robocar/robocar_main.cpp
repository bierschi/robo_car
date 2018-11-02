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

    car->turnCameraStraight();
    sleep(1);
    car->turnCameraXRight(50);
    sleep(1);
    car->turnCameraStraight();
    sleep(1);
    car->turnCameraXLeft(120);
    sleep(1);
    car->turnCameraXRight(30);
    sleep(1);
    car->turnCameraStraight();

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
