//
// Created by christian on 03.09.18.
//

#include <iostream>

#include "car/Car.h"
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
    car->driveForward(40);
    bool run = true;
    int i = 0;
    while(run) {
        car->driveForward(60);
        sleep(2);
        i++;
        if (i > 40) {
            car->driveStop();
            run = false;
        }

    }

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
