//
// Created by christian on 10.09.18.
//

#ifndef ROBOCAR_I2C_H
#define ROBOCAR_I2C_H

#include <inttypes.h>
#include <iostream>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>

#define BUFFER_SIZE 0x01

class I2C {

private:

    int i2cBus;
    int i2cAddr;
    int fd;
    char busFile[64];

public:
    I2C(int, int);
    ~I2C();

    uint8_t dataBuffer[BUFFER_SIZE];
    uint8_t readByte(uint8_t);
    uint8_t writeByte(uint8_t, uint8_t);

};
#endif //ROBOCAR_I2C_H
