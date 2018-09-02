//
// Created by christian on 28.08.18.
//
#include <iostream>
#include <cstdio>
#include <cstring>

#include "comm/server.h"

/**
 * Constructor for a Server instance
 *
 * USAGE:
 *      Server serv(2500);
 *
 * @param port_n: port which the server is listen on
 *
 */
Server::Server(unsigned int port_n) : port(port_n), running(false), dataLen(0){

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("Error in opening socket");
        exit(1);
    }

    memset(&serverAddr, 0, sizeof(serverAddr));

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons((uint16_t)port);

}

/**
 * Destructor: calls the stop() method
 */
Server::~Server() {

    stop();
}

/**
 * cleanly stop the server instance
 */
void Server::stop() {

    running = false;
    close(newsockfd);
    close(sockfd);
}

/**
 * check if the server is running
 *
 * @return 'true', if server is running, else 'false'
 */
bool Server::isRunning() {
    return running;
}

/**
 * get the port of the server instance
 *
 * @return unsigned int port number
 */
int Server::getPort() {
    return port;
}

/**
 * call this run method, to start the server
 */
void Server::run() {

    running = true;

    if (bind(sockfd, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0){
        perror("ERROR on binding");
        exit(1);
    }

    listen(sockfd, 5);
    clientLen = sizeof(clientAddr);

    std::cout << "accept" <<std::endl;
    newsockfd = accept(sockfd, (struct sockaddr *) &clientAddr, &clientLen);
    if (newsockfd < 0){

        perror("ERROR on accept");

    }

    while (running) {

            std::cout << "recv" << std::endl;
            recv();
    }

}

/**
 * processes the incoming messages to the server
 */
void Server::recv() {

    memset(buffer, 0, sizeof(buffer));

    dataLen = read(newsockfd, buffer, BUFFER_LEN-1);
    std::cout << "dataLen: " << dataLen << std::endl;
    if (dataLen <= 0 ) {
        perror("ERROR reading from socket");
        stop();
    }

    std::string data(buffer, dataLen);

    std::cout << "message received: " << data <<std::endl;
    actions(data);

}

/**
 * selects a appropriate action, depending on the incoming data string
 *
 * @param data: received string reference
 */
void Server::actions(const std::string &data) {

    if (data.compare(0, 4, "stop") == 0) {

        std::cout << "stop running Server!" << std::endl;
        stop();

    }
    else if (data.compare(0, 4, "send") == 0){

        std::cout << "send message to client!" << std::endl;


    } else if (data.compare(0, 6, "stream") == 0) {

        std::cout << "create stream object!" << std::endl;


    } else if (data.compare(0, 7, "forward") == 0) {

        std::cout << "drive forward!" << std::endl;


    } else if (data.compare(0, 8, "backward") == 0) {

        std::cout << "drive backward!" << std::endl;


    } else if (data.compare(0, 5, "right") == 0) {

        std::cout << "drive right!" << std::endl;


    } else if (data.compare(0, 4, "left") == 0) {

        std::cout << "drive left" << std::endl;

    }
    else {
        std::cout << "get next message from client" << std::endl;
        std::string nextMsg = "get next message from client";

    }

}

/**
 * send messages to the client
 *
 * @param msg: string reference, which will be sent
 */
void Server::send(const std::string& msg) {

    long n  = write(newsockfd, msg.c_str(), msg.size());
    if (n < 0) perror("ERROR in sending to socket");
}