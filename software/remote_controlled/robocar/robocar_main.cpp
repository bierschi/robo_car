//
// Created by christian on 03.09.18.
//

#include <iostream>

#include "car/Car.h"
#include "server/Server.h"
#include "ros/ros.h"

#define LOGGING true


int main(int argc, char** argv) {

    if (!LOGGING) {
        std::clog.setstate(std::ios_base::failbit);
    }

    // ros initialization
    ros::init(argc, argv, "robocar");

    std::cout << "Software RoboCar is running!" << std::endl;

    Car* car = new Car();
    Server* server = new Server(2501, *car);



    while(ros::ok()) {


            sleep(1);
            ros::spinOnce();
        }

    return 0;

}
