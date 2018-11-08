//
// Created by christian on 14.09.18.
//

#include "car/sensors/Ultrasonic.h"
#include <iostream>
#include <thread>


/**
 * Constructor for a Ultrasonic instance
 *
 * USAGE:
 *      Ultrasonic* us = new Ultrasonic(4, 5);
 *
 * ATTENTION: WIRINGPI pin numbering is used!
 *
 * @param TriggerPin: Int gpio number
 * @param EchoPin Int gpio number
 */
Ultrasonic::Ultrasonic(int TriggerPin, int EchoPin, GearMotor& gearMotor) : Trigger(TriggerPin),
                                                                            Echo(EchoPin),
                                                                            gearMotor_(gearMotor),
                                                                            forwardForbiddenFlag(false),
                                                                            distanceFlag(true)
{

    wiringPiSetup();

    pinMode(Trigger, OUTPUT);
    pinMode(Echo, INPUT);

    digitalWrite(Trigger, LOW);

    distance = getCurrentDistance();
    //gearMotor = new GearMotor(26, 21);

    std::thread distThread(&Ultrasonic::distanceThread, this);
    distThread.detach();

}

/**
 * Destructor in Ultrasonic
 */
Ultrasonic::~Ultrasonic() {
    //delete gearMotor;
}

/**
 * returns the current distance in cm from the ultrasonic range sensor
 *
 * @return distance: double
 */
double Ultrasonic::getCurrentDistance() {

    std::chrono::steady_clock::time_point pulseStart;
    std::chrono::steady_clock::time_point pulseEnd;
    double pulseDuration, distance;

    digitalWrite(Trigger, HIGH);
    usleep(10);
    digitalWrite(Trigger, LOW);

    while (digitalRead(Echo) == LOW) {
        pulseStart = std::chrono::steady_clock::now();
    }

    while (digitalRead(Echo) == HIGH) {
        pulseEnd = std::chrono::steady_clock::now();
    }

    pulseDuration = std::chrono::duration_cast<std::chrono::microseconds>(pulseEnd - pulseStart).count();

    distance = pulseDuration * 1e-6 * 17150;

    //distance = std::floor((distance * 100) + .5) / 100;
    return distance;
}

/**
 * get the bool flag for the DistanceThread
 *
 * @return: bool distanceFlag
 */
bool Ultrasonic::getDistanceThreadFlag() const {
    return distanceFlag;
}

/**
 * set the bool flag for the DistanceThread
 *
 * @param flag: bool
 */
void Ultrasonic::setDistanceThreadFlag(bool flag) {
    distanceFlag = flag;
}

/**
 * distance run Thread to ensure not to hit any obstacle
 */
void Ultrasonic::distanceThread() {

    while ( getDistanceThreadFlag() ) {

        distance = getCurrentDistance();

        if (distance < 12.0) {

            if ( (gearMotor_.getDirection() == 0) && (!forwardForbiddenFlag) ) {

                gearMotor_.setSpeed(0);
                forwardForbiddenFlag = true;
            }

        }

        if (distance > 12.0){

            forwardForbiddenFlag = false;

        }
        //sleep(1);
        //std::cout << "distance: " << distance <<std::endl;
        usleep(400);


    }
}

/**
 * get the ForwardForbiddenFlag, to check if it is allowed to drive forward
 *
 * @return bool: forwardForbiddenFlag
 */
bool Ultrasonic::getForwardForbiddenFlag() const {
    return forwardForbiddenFlag;
}

/**
 * set the ForwardForbiddenFlag
 *
 * @param flag: bool
 */
void Ultrasonic::setForwadForbiddenFlag(bool flag) {
    forwardForbiddenFlag = flag;
}