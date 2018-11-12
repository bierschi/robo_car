//
// Created by christian on 01.11.18.
//

#ifndef ROBOTIC_CAR_ROBOTICCAR_H
#define ROBOTIC_CAR_ROBOTICCAR_H

#include "sensors/SteeringServo.h"
#include "sensors/CameraServo.h"
#include "sensors/GearMotor.h"
#include "sensors/Ultrasonic.h"


class RoboticCar {

private:

    SteeringServo* steeringServo;
    CameraServo* cameraServo;
    GearMotor* gearmotor;
    Ultrasonic* ultrasonic;

public:
    RoboticCar();
    ~RoboticCar();

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
    /*
    //moving camera
    void turnCameraLeft();
    void turnCameraAhead();
    void turnCameraRight();

    //ultrasonic
    void getUltrasonicDistance();
    */
};
#endif //ROBOTIC_CAR_ROBOTICCAR_H
