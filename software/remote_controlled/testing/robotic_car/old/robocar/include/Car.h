//
// Created by christian on 02.11.18.
//

#ifndef ROBOCAR_CAR_H
#define ROBOCAR_CAR_H

#include "sensors/SteeringServo.h"
#include "sensors/CameraServo.h"
#include "sensors/GearMotor.h"
#include "sensors/Ultrasonic.h"

/**
 * /CLASS Car
 *
 * creates a car object to initialize the hardware modules
 */
class Car {

private:

    SteeringServo* steeringServo;
    CameraServo* cameraServo;
    GearMotor* gearmotor;
    Ultrasonic* ultrasonic;

public:
    Car();
     ~Car();

    //Steering actions
    void driveLeft();
    void turnSteeringLeft(int x);
    void driveRight();
    void turnSteeringRight(int x);
    void driveStraight();

    //moving vehicle
    void driveForward(int velocity);
    void driveBackward(int velocity);
    void driveStop();

    //moving camera
    void turnCameraLeft();
    void turnCameraXLeft(int x);
    void turnCameraRight();
    void turnCameraXRight(int x);
    void turnCameraStraight();

    /*
    //ultrasonic
    void getUltrasonicDistance();
    */
};

#endif //ROBOCAR_CAR_H
