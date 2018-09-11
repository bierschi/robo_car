//
// Created by christian on 10.09.18.
//

#ifndef ROBOCAR_I2C_H
#define ROBOCAR_I2C_H

#include <cinttypes>
#include <iostream>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>

#define BUFFER_SIZE 0x01

/**
 * /CLASS I2C
 *
 * creates a I2C object to communicate with the i2c bus
 */
class I2C {

private:

    int i2cAddr, fd;
    unsigned int i2cDevNr;
    char devName[64];
    uint8_t dataBuffer[BUFFER_SIZE];

public:

    I2C(unsigned int devNr, int addr);
    ~I2C();

    uint8_t read(uint8_t);
    uint8_t write(uint8_t, uint8_t);

};
#endif //ROBOCAR_I2C_H
