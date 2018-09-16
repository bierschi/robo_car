//
// Created by christian on 14.09.18.
//

#ifndef ROBOCAR_ULTRASONIC_H
#define ROBOCAR_ULTRASONIC_H

#include <wiringPi.h>
#include <time.h>
#include <unistd.h>
#include <chrono>

/**
 * /CLASS Ultrasonic
 *
 * creates a Ultrasonic object to trigger measurements from an ultrasonic range module
 */
class Ultrasonic {

private:
    int Trigger, Echo;

public:
    Ultrasonic(int TriggerPin, int EchoPin);
    ~Ultrasonic();

    double triggerOneMeasurement();
    void startMeasurement();

};

#endif //ROBOCAR_ULTRASONIC_H
