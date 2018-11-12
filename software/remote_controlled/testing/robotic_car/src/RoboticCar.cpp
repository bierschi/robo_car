//
// Created by christian on 01.11.18.
//

#include "RoboticCar.h"


RoboticCar::RoboticCar()
{

    steeringServo = new SteeringServo(15);
    cameraServo   = new CameraServo(14);
    ultrasonic    = new Ultrasonic(4, 5);
    gearmotor     = new GearMotor(26, 21);

}


RoboticCar::~RoboticCar() {

    delete steeringServo, cameraServo, ultrasonic, gearmotor;

}

//handling the steering servo
void RoboticCar::driveLeft() {
    steeringServo->driveLeft();
}

void RoboticCar::turnSteeringLeft(int x) {
    steeringServo->driveXLeft(x);
}

void RoboticCar::driveRight() {
    steeringServo->driveRight();
}

void RoboticCar::turnSteeringRight(int x) {
    steeringServo->driveXRight(x);
}

void RoboticCar::driveStraight() {
    steeringServo->driveStraight();
}


//handling the for- and backward movements
void RoboticCar::driveForward(int velocity) {
    gearmotor->driveForward(velocity);
}

void RoboticCar::driveBackward(int velocity) {
    gearmotor->driveBackward(velocity);
}

void RoboticCar::driveStop() {
    gearmotor->stop();
}

