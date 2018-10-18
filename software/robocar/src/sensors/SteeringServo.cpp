//
// Created by christian on 16.10.18.
//

#include "sensors/SteeringServo.h"

/**
 * Constructor for a SteeringServo instance
 *
 * USAGE:
 *      SteeringServo* ss = new SteeringServo(15);
 *
 * @param chan: connected channel number of the steering servo
 */
SteeringServo::SteeringServo(uint8_t chan) : PCA9685(1, 0x40, 60), channel(chan){


}

/**
 * Destructor in SteeringServo
 */
SteeringServo::~SteeringServo() {

}

/**
 *moves the steering servo to the left side
 */
void SteeringServo::driveLeft() {

    PCA9685::setPWM(channel, 1750, 1895);

}

/**
 * moves the steering servo to the right side
 */
void SteeringServo::driveRight() {

    PCA9685::setPWM(channel, 1230, 1720);

}

/**
 * moves the steering servo to the middle
 */
void SteeringServo::driveStraight() {

    PCA9685::setPWM(channel, 1770, 2130); //1750-1770

}
