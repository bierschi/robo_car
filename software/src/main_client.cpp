//
// Created by christian on 28.08.18.
//

#include <iostream>
#include "cstring"
#include "cstdio"
#include "comm/client.h"

int main() {
    std::cout << "Test Client Class" << std::endl;

    Client client("localhost", 2503);
    client.run();

    return 0;
}

