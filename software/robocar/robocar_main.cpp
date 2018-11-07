//
// Created by christian on 03.09.18.
//

#include <iostream>

//#include "car/Car.h"
//#include "server/Server.h"
#include "slam/SlamMap.h"
#include "ros/ros.h"

#define LOGGING true


void writeFile(int mapData[]) {
    FILE* out = fopen("intarray_SlamMap.txt", "w");
    for( int s = 1; s < 160000; s++) {

        fprintf(out, "%d ", mapData[s]);

        if (s && s%400 == 0) {

            fprintf(out, "\n");

        }
    }
}

int main(int argc, char** argv) {

    if (!LOGGING) {
        std::clog.setstate(std::ios_base::failbit);
    }

    // ros initialization
    ros::init(argc, argv, "robocar");

    std::cout << "Software RoboCar is running!" << std::endl;

    SlamMap sm;

    //Car* car = new Car();
    //Server* server = new Server(2501, *car);

    //int* data = new int[160000];
    sm.setSaveMap(true);
    int* data;
    while(ros::ok()) {

            data = sm.getMapData(); //+size
            std::cout << data[10000] << std::endl;
            writeFile(data);
            sleep(2);
            ros::spinOnce();

        }

    return 0;

}
