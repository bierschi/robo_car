//
// Created by christian on 10.09.18.
//
#include "comm/I2C.h"

/**
 * Constructor for a I2C instance
 *
 * USAGE:
 *      I2C *ic2 = new I2C(1, 0x40)
 *
 * @param devNr: unsigned int
 * @param addr: int address for i2c slave
 */
I2C::I2C(unsigned int devNr, int addr) : i2cDevNr(devNr), i2cAddr(addr) {

    std::snprintf(devName, sizeof(devName), "/dev/i2c-%d", i2cDevNr);

    if ((fd = open(devName, O_RDWR)) < 0) {

        throw std::runtime_error("Could not open I2C Device!");

    }

    if (ioctl(fd, I2C_SLAVE, i2cAddr) < 0) {

        throw std::runtime_error("Accessing I2C slave failed!");
    }

}

/**
 * Destructor in I2C
 */
I2C::~I2C() {

    close(fd);

}

/**
 * read from i2c bus with param 'address'
 *
 * @param address: uint8_t address to read from
 */
uint8_t I2C::read(uint8_t address) {

    if (fd != -1) {

        uint8_t buff[BUFFER_SIZE];
        buff[0] = address;

        if ( ::write(fd, buff,BUFFER_SIZE) != BUFFER_SIZE) {

            std::clog << "Failed to access I2C slave address" << std::endl;

        } else {

            if (::read(fd, dataBuffer, BUFFER_SIZE) != BUFFER_SIZE) {

                std::clog << "Could not read from I2C slave" << std::endl;

            } else {

                return dataBuffer[0];

            }
        }
    } else {

        std::clog << "Device file not available" << std::endl;

    }
}

/**
 * write to i2c bus with params 'address' and 'data'
 *
 * @param address: uint8_t address to write to
 * @param data: uint8_t data byte to write
 */
uint8_t I2C::write(uint8_t address, uint8_t data) {

    if ( fd != -1 ){

        uint8_t buff[2];
        buff[0] = address;
        buff[1] = data;

        if ( ::write(fd, buff, sizeof(buff)) != 2) {

            std::clog << "Failed to write to I2C slave!" << std::endl;

        } /*else {

            std::clog << "Wrote to I2C Slave " << std::endl;

        }*/

    } else {

        std::clog << "I2C device not available!" << std::endl;
    }

    return 0;
}