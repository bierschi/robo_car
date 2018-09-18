//
// Created by christian on 14.09.18.
//

#ifndef ROBOCAR_ULTRASONIC_H
#define ROBOCAR_ULTRASONIC_H

#include <wiringPi.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <comm/ServerSocket.h>

/**
 * /CLASS Ultrasonic
 *
 * creates a Ultrasonic object to trigger measurements from an ultrasonic range module
 */
class Ultrasonic {

private:
    int Trigger, Echo;
    bool isRunning;

public:
    Ultrasonic(int TriggerPin, int EchoPin);
    ~Ultrasonic();

    void setIsRunning(bool runFlag);
    bool getIsRunning() const;
    double triggerOneMeasurement();
    void continousMeasurement(ServerSocket& sock);

};

#endif //ROBOCAR_ULTRASONIC_H
