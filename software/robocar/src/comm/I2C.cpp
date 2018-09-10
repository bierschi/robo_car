//
// Created by christian on 10.09.18.
//
#include "comm/I2C.h"


I2C::I2C(int bus, int address) : i2cBus(bus), i2cAddr(address) {

    snprintf(busFile, sizeof(busFile), "/dev/i2c-%d", bus);

    if ((fd = open(busFile, O_RDWR)) < 0) {

        throw std::runtime_error("Could not open I2C Bus");

    }

    if (ioctl(fd, I2C_SLAVE, i2cAddr) < 0) {

        throw std::runtime_error("I2C slave failed!");
    }

}

I2C::~I2C() {

    close(fd);

}

uint8_t I2C::readByte(uint8_t address) {

    if (fd != -1) {

        uint8_t buff[BUFFER_SIZE];
        buff[0] = address;

        if ( write(fd, buff,BUFFER_SIZE) != BUFFER_SIZE) {
            std::clog << "I2C slave failed to go to register" << std::endl;
        } else {
            if (read(fd, dataBuffer, BUFFER_SIZE) != BUFFER_SIZE) {
                std::clog << "Could not read from I2C slave" << std::endl;
            } else {
                return dataBuffer[0];
            }
        }
    } else {
        std::clog << "Device file not availalbe" << std::endl;
    }
}


uint8_t I2C::writeByte(uint8_t address, uint8_t data) {

    if ( fd != -1 ){

        uint8_t buff[2];
        buff[0] = address;
        buff[1] = data;

        if ( write(fd, buff, sizeof(buff)) != 2) {

            std::runtime_error("Failed to write to I2C slave!");

        } else {
            std::clog << "Wrote to I2C Slave " << std::endl;
        }

    } else {

        std::clog << "I2C device not available!" << std::endl;
    }

    return 0;
}