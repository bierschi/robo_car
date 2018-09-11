//
// Created by christian on 10.09.18.
//


#include "sensors/PCA9685.h"

/**
 * Constructor for a PCA9685 instance
 *
 * USAGE:
 *      PCA9685 *pca = new PCA9685(1, 0x40, 60);
 *
 * @param devNr: unsigned int number of the i2c device
 * @param address: int address of the i2c slave
 * @param freq: int frequency of the pwm signal
 */
PCA9685::PCA9685(unsigned int devNr, int addr, int freq) : i2cDevNr(devNr), i2cAddr(addr), frequency(freq){

    i2c = new I2C(i2cDevNr, i2cAddr);

    reset();

    if (frequency > 1000 || frequency < 40) {

        throw std::runtime_error("frequency value must be between 40Hz and 1000Hz!");

    } else {

        setPWMFreq(frequency);

    }

}

/**
 * Destructor in PCA9685
 */
PCA9685::~PCA9685() {

    delete i2c;

}

/**
 * resets the PCA9685 module
 */
void PCA9685::reset() {

    i2c->write(MODE1, 0x00);
    i2c->write(MODE2, 0x04);

}

/**
 * sets desired pwm frequency
 *
 * @param freq: int frequency of pwm signal
 */
void PCA9685::setPWMFreq(int freq) {

    freq = (freq > 1000 ? 1000 : (freq < 40 ? 40 : freq));

    uint8_t prescale_val = (int)(CLOCK_FREQ / (4096 * freq) - 0.5f);

    i2c->write(MODE1, 0x10);
    i2c->write(PRE_SCALE, prescale_val);
    i2c->write(MODE1, 0x80);
    i2c->write(MODE2, 0x04);

}

/**
 * sets the pwm signal to a desired channel(0-15)
 *
 * @param channel: uint8_t channel, which should be controlled
 * @param onValue: int onValue of the pwm signal
 * @param offValue: int offValue of the pwm signal
 */
void PCA9685::setPWM(uint8_t channel, int onValue, int offValue) {

    i2c->write(LED0_ON_L  + LED_FACTOR * channel, onValue & 0xFF);
    i2c->write(LED0_ON_H  + LED_FACTOR * channel, onValue >> 8);
    i2c->write(LED0_OFF_L + LED_FACTOR * channel, offValue & 0xFF);
    i2c->write(LED0_OFF_H + LED_FACTOR * channel, offValue >> 8);

}

