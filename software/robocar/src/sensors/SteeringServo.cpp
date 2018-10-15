//
// Created by christian on 16.10.18.
//

#include "sensors/SteeringServo.h"

SteeringServo::SteeringServo(uint8_t chan) : PCA9685(1, 0x40, 60), channel(chan){


}


SteeringServo::~SteeringServo() {

}


void SteeringServo::driveLeft() {

    PCA9685::setPWM(channel, 1750, 1895);

}

void SteeringServo::driveRight() {

    PCA9685::setPWM(channel, 1230, 1720);

}

void SteeringServo::driveStraight() {

    PCA9685::setPWM(channel, 1770, 2130); //1750-1770

}
