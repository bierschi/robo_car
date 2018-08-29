//
// Created by christian on 28.08.18.
//
#include <iostream>
#include <cstdio>
#include <cstring>

#include "server.h"

Server::Server(int port_n) {

    port = port_n;
    running = false;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("Error in opening socket");
    }

    memset(&serv_addr, 0, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons((uint16_t)port);

}

Server::~Server() {

    stop();
}

void Server::stop() {

    running = false;
    close(newsockfd);
    close(sockfd);
}

bool Server::isRunning() {
    return running;
}


int Server::getPort() {
    return port;
}

void Server::run() {

    running = true;

    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0){
        perror("ERROR on binding");
        exit(1);
    }

    listen(sockfd, 5);
    clilen = sizeof(cli_addr);

    while (running) {

        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

        if (newsockfd < 0){

            perror("ERROR on accept");

        } else {

            recv();

        }

    }

}

void Server::recv() {

    memset(buffer, 0, sizeof(buffer));

    dataLen = read(newsockfd, buffer, BUFFER_LEN-1);

    if (dataLen < 0 ) perror("ERROR reading from socket");

    std::string data(buffer, dataLen);

    std::cout << "message received: " << data << std::endl;
    actions(data);

}

void Server::actions(const std::string &data) {

    if (data.compare(0, 4, "stop") == 0) {
        std::cout << "stop running server!" << std::endl;
        stop();
    }
    else if (data.compare(0, 4, "send") == 0){
        std::cout << "send message to client!" << std::endl<<std::endl;
        std::string strToClient = "msg to Client";
        send(strToClient);
    }
    else {
        std::cout << "get next message from client" << std::endl<<std::endl;
    }

}

void Server::send(const std::string& msg) {

    long n  = write(newsockfd, msg.c_str(), msg.size());
    if (n < 0) perror("ERROR in sending to socket");
}