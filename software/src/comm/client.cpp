//
// Created by Christian on 27.08.2018.
//
#include <iostream>

#include "comm/client.h"


Client::Client(std::string &host_n, unsigned int &port_n) :  host(host_n), port(port_n), sockfd(0), connected(false){

    server = gethostbyname(host.data());
    if (server == NULL) {
        perror("ERROR, no such host!");
        exit(1);
    }
    memset(&serv_addr, 0, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    bcopy((char*)server->h_addr, (char*)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons((uint16_t)port);

    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
        perror("Error in opening socket!");
        exit(1);
    }

}

Client::~Client() {
    close(sockfd);
}



void Client::run() {

    if (connect(sockfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0) {
        perror("Error in connecting to socket!");
    }
    connected = true;
}

void Client::recv() {

    n = read(sockfd, buf, 255);
    if (n < 0) {
        perror("ERROR reading to socket!");
    }
}

void Client::send() {

    std::cout << "Please enter a message: ";
    bzero(buf, 256);
    fgets(buf, 255, stdin);
    n = write(sockfd, buf, strlen(buf));
    if (n < 0) {
        perror("ERROR writing to socket!");
    }
}