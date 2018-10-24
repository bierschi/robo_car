//
// Created by christian on 21.09.18.
//

#ifndef ROBOCAR_GEARMOTOR_H
#define ROBOCAR_GEARMOTOR_H

#include <wiringPi.h>

#define MAX_SPEED 480

/**
 * /CLASS GearMotor
 *
 * creates a GearMotor object to control the direction and the speed of a gear motor
 */
class GearMotor {

private:
    int pwmPin, directionPin;
    int speedValue, maxSpeedValue;
    int direction;

public:
    GearMotor(int pwmP, int directionP);
    ~GearMotor();

    void setSpeed(int value);
    int getSpeed() const;
    int getMaxSpeed() const;
    int getDirection() const;

    void stop();
    void fast();
    void middle();
    void slow();

};

#endif //ROBOCAR_GEARMOTOR_H
