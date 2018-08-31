#include <iostream>
#include "cstdio"

//#include "opencv2/opencv.hpp"
#include "comm/server.h"

int main() {
    std::cout << "Test Server Class" << std::endl;

    Server serv(2503);
    serv.run();
    serv.stop();

    return 0;
}

