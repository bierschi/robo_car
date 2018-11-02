//
// Created by christian on 02.11.18.
//

#include "Car.h"

/**
 *
 */
Car::Car() {

    steeringServo = new SteeringServo(15);
    cameraServo   = new CameraServo(14);
    ultrasonic    = new Ultrasonic(4, 5);
    gearmotor     = new GearMotor(26, 21);
}

/**
 *
 */
Car::~Car() {

    delete steeringServo, cameraServo, ultrasonic, gearmotor;

}


//handling the steering servo
/**
 *
 */
void Car::driveLeft() {
    steeringServo->driveLeft();
}

/**
 *
 * @param x
 */
void Car::turnSteeringLeft(int x) {
    steeringServo->driveXLeft(x);
}

/**
 *
 */
void Car::driveRight() {
    steeringServo->driveRight();
}

/**
 *
 * @param x
 */
void Car::turnSteeringRight(int x) {
    steeringServo->driveXRight(x);
}

/**
 *
 */
void Car::driveStraight() {
    steeringServo->driveStraight();
}

//handling the for- and backward movements
/**
 *
 * @param velocity
 */
void Car::driveForward(int velocity) {
    gearmotor->driveForward(velocity);
}

/**
 *
 * @param velocity
 */
void Car::driveBackward(int velocity) {
    gearmotor->driveBackward(velocity);
}

/**
 *
 */
void Car::driveStop() {
    gearmotor->stop();
}