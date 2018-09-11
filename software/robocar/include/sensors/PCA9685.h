//
// Created by christian on 10.09.18.
//

#ifndef ROBOCAR_PCA9685_H
#define ROBOCAR_PCA9685_H

#include "comm/I2C.h"

// register definitions
// checkout datasheet for further informations
// https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define MODE1 0x00
#define MODE2 0x01

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define LED_FACTOR 4
#define PRE_SCALE 0xFE
#define CLOCK_FREQ 25000000.0

/**
 * /CLASS PCA9685
 *
 * creates a PCA9685 object to communicate with the servomotor board PCA9685
 */
class PCA9685 {

private:
    I2C *i2c;
    int i2cAddr;
    unsigned int i2cDevNr;
    int frequency;

public:

    PCA9685(unsigned int devNr, int addr, int freq);
    ~PCA9685();

    void reset();
    void setPWMFreq(int freq);
    void setPWM(uint8_t channel, int onValue, int offValue);

};

#endif //ROBOCAR_PCA9685_H
