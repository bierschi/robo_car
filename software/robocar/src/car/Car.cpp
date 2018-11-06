//
// Created by christian on 02.11.18.
//

#include "car/Car.h"

/**
 * Constructor for a Car instance
 *
 * USAGE:
 *      Car* car = new Car();
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
 * moves the steering servo maximal to the left
 */
void Car::driveLeft() {
    steeringServo->driveLeft();
}

/**
 * moves the steering servo x% to the left
 *
 * @param x: int number between 0 - 100
 */
void Car::turnSteeringLeft(int x) {
    steeringServo->driveXLeft(x);
}

/**
 * moves the steering servo maximal to the right
 */
void Car::driveRight() {
    steeringServo->driveRight();
}

/**
 * moves the steering servo x% to the right
 *
 * @param x: int number between 0 - 100
 */
void Car::turnSteeringRight(int x) {
    steeringServo->driveXRight(x);
}

/**
 * moves the steering servo to the middle -> drive straight ahead
 */
void Car::driveStraight() {
    steeringServo->driveStraight();
}

//handling the for- and backward movements
/**
 * moves the car with x% forward
 *
 * @param velocity: int number between 0 - 100
 */
void Car::driveForward(int velocity) {

    if ( !ultrasonic->getForwardForbiddenFlag() )
        gearmotor->driveForward(velocity);
}

/**
 * moves the car with x% backward
 *
 * @param velocity: int number between 0 - 100
 */
void Car::driveBackward(int velocity) {
    gearmotor->driveBackward(velocity);
}

/**
 * stops the car
 */
void Car::driveStop() {
    gearmotor->stop();
}


//handling the camera servo
/**
 * moves the camera servo a bit to the left
 */
void Car::turnCameraLeft() {
    cameraServo->moveLeft();
}

/**
 * moves the camera servo x% to the left
 *
 * @param x: int number between 0 - 100
 */
void Car::turnCameraXLeft(int x) {
    cameraServo->moveXLeft(x);
}

/**
 * moves the camera servo a bit to the right
 */
void Car::turnCameraRight() {
    cameraServo->moveRight();
}

/**
 * moves the camera servo x% to the right
 *
 * @param x: int number between 0 - 100
 */
void Car::turnCameraXRight(int x) {
    cameraServo->moveXRight(x);
}

/**
 * moves the camera servo to the middle
 */
void Car::turnCameraStraight() {
    cameraServo->moveStraight();
}