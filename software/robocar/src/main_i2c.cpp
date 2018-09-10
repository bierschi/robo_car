//
// Created by christian on 10.09.18.
//

#include "comm/I2C.h"
#include "sensors/PCA9685.h"

int main(int argc, char* argv[]) {

    std::cout << "PCA test" << std::endl;

    PCA9685 pca(1, 0x40);






    return 0;
}