//
// Created by christian on 10.09.18.
//


#include "sensors/PCA9685.h"


PCA9685::PCA9685(unsigned int devNr, int address) {

    i2c = new I2C(devNr, address);

    setPWMFreq(60);
}


PCA9685::~PCA9685() {

    delete i2c;

}

void PCA9685::setPWMFreq(int freq) {

    uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq) - 1;

    i2c->writeByte(MODE1, 0x10);
    i2c->writeByte(PRE_SCALE, prescale_val);
    i2c->writeByte(MODE1, 0x80);
    i2c->writeByte(MODE2, 0x04);

}

void PCA9685::setPWM(uint8_t channel, int onValue, int offValue) {

    i2c->writeByte(LED0_ON_L + LED_MULTIPLYER * (channel -1 ), onValue & 0xFF);
    i2c->writeByte(LED0_ON_H + LED_MULTIPLYER * (channel -1), onValue >> 8);
    i2c->writeByte(LED0_OFF_L + LED_MULTIPLYER * (channel -1), offValue & 0xFF);
    i2c->writeByte(LED0_OFF_H + LED_MULTIPLYER * (channel -1), offValue >> 8);

}

