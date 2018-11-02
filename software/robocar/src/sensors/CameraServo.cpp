//
// Created by christian on 18.10.18.
//

#include "sensors/CameraServo.h"

/**
 * Constructor for a CameraServo instance
 *
 * USAGE:
 *      CameraServo* cs = new CameraServo(14);
 *
 * @param chan: connected channel number of the camera servo
 */
CameraServo::CameraServo(uint8_t chan) : PCA9685(1, 0x40, 60),
                                         channel(chan),
                                         countCamServo(2115),
                                         xLeft(2115),
                                         xRight(2115)
{


}

/**
 * Destructor in CameraServo
 */
CameraServo::~CameraServo() {

}

/**
 * moves the camera servo to the right side
 */
void CameraServo::moveRight() {

    if (countCamServo > 1820){

        countCamServo -= 30;
        PCA9685::setPWM(channel, 1750, countCamServo);

    }

}

/**
 * moves the camera servo x% to the right side (0% - 100%)
 *
 * @param x: int value between 0 and 100
 */
void CameraServo::moveXRight(int x) {

    if (x >= 100) {
        xRight = 1820;
    } else if (x <= 0) {
        xRight = 2115;
    } else {
        xRight = xRight - (2.95 * x);
    }

    PCA9685::setPWM(channel, 1750, (int)xRight);
    //set default value
    xRight = 2115;

}

/**
 * moves the camera servo to the left side
 */
void CameraServo::moveLeft() {

    if (countCamServo < 2320){

        countCamServo += 30;
        PCA9685::setPWM(channel, 1750, countCamServo);

    }

}

/**
 * moves the camera servo x% to the left side (0% - 100%)
 *
 * @param x: int value between 0 and 100
 */
void CameraServo::moveXLeft(int x) {

    if (x >= 100) {
        xLeft = 2320;
    } else if (x <= 0) {
        xLeft = 2115;
    } else {
        xLeft = xLeft + (2.1 * x);
    }

    PCA9685::setPWM(channel, 1750, (int)xLeft);
    //set default value
    xLeft = 2115;
}

/**
 * moves the camera servo to the middle
 */
void CameraServo::moveStraight() {

    PCA9685::setPWM(channel, 1750, countCamServo);

}