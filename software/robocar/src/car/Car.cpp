//
// Created by christian on 02.11.18.
//

#include "car/Car.h"

/**
 * Constructor for a Car instance
 *
 * USAGE:
 *      Car *car = new Car();
 *
 */
Car::Car() {

    steeringServo = new SteeringServo(15);
    cameraServo   = new CameraServo(14);
    gearmotor     = new GearMotor(26, 21);
    ultrasonic    = new Ultrasonic(4, 5);

}

/**
 * Destructor in Car
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

    if ( !ultrasonic->getForwardForbiddenFlag() )
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


//handling the camera servo
/**
 *
 */
void Car::turnCameraLeft() {
    cameraServo->moveLeft();
}

/**
 *
 * @param x
 */
void Car::turnCameraXLeft(int x) {
    cameraServo->moveXLeft(x);
}

/**
 *
 */
void Car::turnCameraRight() {
    cameraServo->moveRight();
}

/**
 *
 * @param x
 */
void Car::turnCameraXRight(int x) {
    cameraServo->moveXRight(x);
}

/**
 *
 */
void Car::turnCameraStraight() {
    cameraServo->moveStraight();
}