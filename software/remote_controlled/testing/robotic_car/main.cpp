//
// Created by christian on 01.11.18.
//

#include "RoboticCar.h"


int main(int argc, char** argv) {

    RoboticCar* rc = new RoboticCar();

    rc->driveStraight();
    sleep(2);
    rc->driveBackward(10);
    sleep(1);
    rc->driveStop();
    sleep(1);
    rc->driveForward(120);
    sleep(1);
    rc->driveStop();
    sleep(1);
    rc->driveBackward(40);
    sleep(1);
    rc->driveForward(60);
    sleep(1);
    rc->driveStop();

    return 0;
}
