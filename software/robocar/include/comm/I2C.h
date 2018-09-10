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

class I2C {

private:

    int i2cAddr, fd;
    unsigned int i2cDevNr;
    char devName[64];

public:
    I2C(unsigned int, int);
    ~I2C();

    uint8_t dataBuffer[BUFFER_SIZE];
    uint8_t readByte(uint8_t);
    uint8_t writeByte(uint8_t, uint8_t);

};
#endif //ROBOCAR_I2C_H
