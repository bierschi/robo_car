//
// Created by christian on 18.10.18.
//

#ifndef ROBOCAR_CAMERASERVO_H
#define ROBOCAR_CAMERASERVO_H

#include "sensors/PCA9685.h"

/**
 * /CLASS CameraServo
 *
 * creates a CameraServo object to control the camera servo
 */
class CameraServo : public PCA9685{

private:
    uint8_t channel;
    int countCamServo;

public:
    CameraServo(uint8_t chan);
    ~CameraServo();

    void moveLeft();
    void moveRight();
    void moveStraight();

};

#endif //ROBOCAR_CAMERASERVO_H
