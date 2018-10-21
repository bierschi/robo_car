//
// Created by christian on 16.10.18.
//

#ifndef ROBOCAR_GUI_STEERINGSERVO_H
#define ROBOCAR_GUI_STEERINGSERVO_H

#include "PCA9685.h"

/**
 * /CLASS SteeringServo
 *
 * creates a SteeringServo object to control the steering servo
 */
class SteeringServo : public PCA9685 {

private:
    uint8_t channel;

public:
    SteeringServo(uint8_t chan);
    ~SteeringServo();

    void driveLeft();
    void driveRight();
    void driveStraight();

};

#endif //ROBOCAR_GUI_STEERINGSERVO_H