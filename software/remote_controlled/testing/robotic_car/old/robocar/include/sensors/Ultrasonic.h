//
// Created by christian on 14.09.18.
//

#ifndef ROBOCAR_ULTRASONIC_H
#define ROBOCAR_ULTRASONIC_H

#include <wiringPi.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <cmath>

#include "sensors/GearMotor.h"

/**
 * /CLASS Ultrasonic
 *
 * creates a Ultrasonic object to trigger measurements from an ultrasonic range module
 */
class Ultrasonic {

private:
    int Trigger, Echo;
    double distance;
    bool forwardForbiddenFlag, distanceFlag;
    GearMotor* gearMotor;

public:
    Ultrasonic(int TriggerPin, int EchoPin);
    ~Ultrasonic();

    bool getForwardForbiddenFlag() const;
    void setForwadForbiddenFlag(bool flag);

    double getCurrentDistance();

    //methods to manipulate the distance thread
    void setDistanceThreadFlag(bool flag);
    bool getDistanceThreadFlag() const;
    void distanceThread();

};

#endif //ROBOCAR_ULTRASONIC_H
