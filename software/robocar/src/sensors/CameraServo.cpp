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
CameraServo::CameraServo(uint8_t chan) : PCA9685(1, 0x40, 60), channel(chan), countCamServo(2120){


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
 * moves the camera servo to the left side
 */
void CameraServo::moveLeft() {

    if (countCamServo < 2320){

        countCamServo += 30;
        PCA9685::setPWM(channel, 1750, countCamServo);

    }

}

/**
 * moves the camera servo to the middle
 */
void CameraServo::moveStraight() {

    PCA9685::setPWM(channel, 1750, countCamServo);

}