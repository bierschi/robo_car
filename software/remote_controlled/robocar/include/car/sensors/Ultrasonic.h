//
// Created by christian on 14.09.18.
//

#ifndef ROBOCAR_ULTRASONIC_H
#define ROBOCAR_ULTRASONIC_H

#include <wiringPi.h>
#include "../../../../../../../../../usr/include/time.h"
#include "../../../../../../../../../usr/include/unistd.h"
#include "../../../../../../../../../usr/include/c++/5/chrono"
#include "../../../../../../../../../usr/include/c++/5/thread"
#include "../../../../../../../../../usr/include/c++/5/cmath"

#include "GearMotor.h"

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
    GearMotor& gearMotor_;

public:
    Ultrasonic(int TriggerPin, int EchoPin, GearMotor& gearMotor);
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
