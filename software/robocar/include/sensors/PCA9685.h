//
// Created by christian on 10.09.18.
//

#ifndef ROBOCAR_PCA9685_H
#define ROBOCAR_PCA9685_H

#include "comm/I2C.h"

#define MODE1 0x00
#define MODE2 0x01

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define LED_MULTIPLYER 4
#define PRE_SCALE 0xFE
#define CLOCK_FREQ 25000000.0

class PCA9685 {

private:
    I2C *i2c;


public:
    PCA9685(unsigned int, int);
    ~PCA9685();

    void setPWMFreq(int freq);
    void setPWM(uint8_t channel, int onValue, int offValue);

};

#endif //ROBOCAR_PCA9685_H
