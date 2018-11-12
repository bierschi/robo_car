//
// Created by christian on 16.10.18.
//

#include "car/sensors/SteeringServo.h"

/**
 * Constructor for a SteeringServo instance
 *
 * USAGE:
 *      SteeringServo* ss = new SteeringServo(15);
 *
 * @param chan: connected channel number of the steering servo
 */
SteeringServo::SteeringServo(uint8_t chan) : PCA9685(1, 0x40, 60),
                                             channel(chan),
                                             xLeft(2100),
                                             xRight(1600)
{


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
 * moves the steering servo between 0% - 100%, where 0% is straight ahead and 100% is driveLeft()
 *
 * @param x: int value between 0 and 100
 */
void SteeringServo::driveXLeft(int x) {

    if (x >= 100) {
        xLeft = 1900;
    } else if (x <= 0) {
        xLeft = 2100;
    } else {
        xLeft = xLeft - (2*x);
    }

    PCA9685::setPWM(channel, 1750, xLeft);
    //set default value
    xLeft = 2100;
}

/**
 * moves the steering servo to the right side
 */
void SteeringServo::driveRight() {

    PCA9685::setPWM(channel, 1230, 1720);

}

/**
 * moves the steering servo between 0% - 100%, where 0% is straight ahead and 100% is driveRight()
 *
 * @param x: int value between 0 and 100
 */
void SteeringServo::driveXRight(int x) {

    if (x >= 100) {
        xRight = 1720;
    } else if (x <= 0) {
        xRight = 1600;
    } else {
        xRight = xRight + (1.2*x);
    }
    xRight = round(xRight);

    PCA9685::setPWM(channel, 1230, (int)xRight);
    //set default value
    xRight = 1600;
}

/**
 * moves the steering servo to the middle
 */
void SteeringServo::driveStraight() {

    PCA9685::setPWM(channel, 1770, 2130); //1750-1770

}
