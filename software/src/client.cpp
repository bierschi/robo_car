//
// Created by Christian on 27.08.2018.
//
#include "client.h"
#include <string.h>
#include <afxres.h>

Client::Client(std::string &host, unsigned int &port) {
    this->host = host;
    this->port = port;

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;

}

Client::~Client() {}

void Client::connect() {

}

std::string Client::read() {

}

void Client::send() {

}