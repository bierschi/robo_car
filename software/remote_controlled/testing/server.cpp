//
// Created by christian on 28.08.18.
//
#include <iostream>
#include <cstdio>
#include <cstring>

#include "server.h"

/**
 * Constructor for a Server instance
 *
 * USAGE:
 *      Server serv(2500);
 *
 * @param port_n: port which the server is listen on
 *
 */

void *PrintHello(void *threadid) {
    long tid;
    tid = (long)threadid;
    std::cout << "Hello World! Thread ID, " << tid << std::endl;
    pthread_exit(NULL);
}

Server::Server(unsigned int port_n) : port(port_n), running(false), dataLen(0){

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {

        throw std::runtime_error("Failed to create socket");

    }

    memset(&serverAddr, 0, sizeof(serverAddr));

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons((uint16_t)port);

    if (bind(sockfd, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0){

      throw std::runtime_error("Failed to bind to port");

    }

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
void Server::waitForConnection() {

    running = true;

    int res = listen(sockfd, 5);
    if (res < 0 ) {

        throw std::runtime_error("Listen failed");

    }

    clientLen = sizeof(clientAddr);

    std::cout << "Listening ...!" <<std::endl;
    newsockfd = accept(sockfd, (struct sockaddr *) &clientAddr, &clientLen);
    if (newsockfd < 0){

        throw std::runtime_error("accept failed");

    }

    std::thread serverThread(&Server::handleConnection, this);
    serverThread.join();
    /*
    while (running) {

            std::cout << "recv" << std::endl;
            recv();
    }
     */

}

/**
 *
 */
void Server::handleConnection() {

    std::cout << "start thread" << std::endl;
    //std::cout << arg;
    int i = 0;
    while (i < 5) {
        std::cout << i << std::endl;
        recv();
        i++;
    }

}

/**
 * processes the incoming messages to the server
 */
void Server::recv() {

    memset(buffer, 0, sizeof(buffer));

    dataLen = read(newsockfd, buffer, BUFFER_LEN);
    //std::cout << "dataLen: " << dataLen << std::endl;
    if (dataLen <= 0 ) {
        perror("ERROR reading from socket");
        stop();
    }

    //std::string data(buffer, dataLen);
    std::string data(buffer);

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