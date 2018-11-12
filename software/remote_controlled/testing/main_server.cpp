#include <iostream>
#include "cstdio"

//#include "opencv2/opencv.hpp"
#include "server.h"


int main() {
    std::cout << "Test Server Class" << std::endl;


    Server serv(2500);
    serv.waitForConnection();
    //serv.stop();

    return 0;
}

