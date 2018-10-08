//
// Created by christian on 21.09.18.
//

#include "sensors/GearMotor.h"

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
GearMotor::GearMotor(int pwmP, int directionP) : pwmPin(pwmP), directionPin(directionP), speedValue(0), maxSpeedValue(MAX_SPEED){

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

    int direction;

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