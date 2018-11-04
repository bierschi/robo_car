//
// Created by christian on 21.09.18.
//

#include "car/sensors/GearMotor.h"

/**
 * Constructor for a GearMotor instance
 *
 * USAGE:
 *      GearMotor* gm = new GearMotor(12, 5);
 *
 * ATTENTION: WIRINGPI pin numbering is used!
 *
 * @param pwmP: Int PWM GPIO number
 * @param directionP: Int GPIO number
 */
GearMotor::GearMotor(int pwmP, int directionP) : pwmPin(pwmP),
                                                 directionPin(directionP),
                                                 speedValue(0),
                                                 maxSpeedValue(MAX_SPEED),
                                                 direction(-1),
                                                 forwardVelocity_(0),
                                                 backwardVelocity_(0)
{

    wiringPiSetup();

    pinMode(pwmPin, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(MAX_SPEED);
    pwmSetClock(2);

    pinMode(directionPin, OUTPUT);


}
/**
 * Destructor in GearMotor
 */
GearMotor::~GearMotor() {

}

/**
 * set speed (height of the number) and direction (positive or negative number) of the gear motor
 *
 * @param value: int value
 */
void GearMotor::setSpeed(int value) {

    //int direction;

    if (value < 0){

        speedValue = -value;
        direction = 1;

    } else {

        speedValue = value;
        direction = 0;

    }

    if (value > MAX_SPEED)

        speedValue = MAX_SPEED;

    digitalWrite(directionPin, direction);
    pwmWrite(pwmPin, speedValue);

}

int GearMotor::getDirection() const {
    return direction;
}

/**
 * return current speed value
 *
 * @return int speedValue
 */
int GearMotor::getSpeed() const {
    return speedValue;
}

/**
 * return maximal speed value
 *
 * @return int maxSpeedValue
 */
int GearMotor::getMaxSpeed() const {
    return maxSpeedValue;
}

/**
 *
 * @param velocity
 */
void GearMotor::driveForward(int velocity) {

    if (velocity >= 100) {
        forwardVelocity_ = MAX_SPEED;
    } else if (velocity <= 0) {
        forwardVelocity_ = 0;
    } else {
        forwardVelocity_ = 4.8 * velocity;
    }

    setSpeed((int)forwardVelocity_);
    //set default value
    forwardVelocity_ = 0;

}

/**
 *
 * @param velocity
 */
void GearMotor::driveBackward(int velocity) {

    if (velocity >= 100) {
        backwardVelocity_ = -MAX_SPEED;
    } else if (velocity <= 0) {
        backwardVelocity_ = 0;
    } else {
        backwardVelocity_ = - (4.8 * velocity);
    }

    setSpeed((int)backwardVelocity_);
    //set default value
    backwardVelocity_ = 0;
}


/**
 * stops the GearMotor
 */
void GearMotor::stop() {

    setSpeed(0);

}

/**
 * sets the speed to MAX_SPEED
 */
void GearMotor::fast() {

    setSpeed(MAX_SPEED);

}

/**
 * sets the speed to half of MAX_SPEED
 */
void GearMotor::middle() {

    setSpeed(MAX_SPEED/2);

}

/**
 * sets the speed to slow
 */
void GearMotor::slow() {

    setSpeed(70);

}