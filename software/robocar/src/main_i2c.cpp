//
// Created by christian on 10.09.18.
//

#include "comm/I2C.h"

int main(int argc, char* argv[]) {

    I2C i2c(1, 0x20);

    std::cout << "write to bus" << std::endl;

    i2c.writeByte(0x00, 0x10);

    return 0;
}