//
// Created by Christian on 27.08.2018.
//
#include <iostream>

#include "client.h"

/**
 * Constructor for a Client object
 *
 * USAGE:
 *      Client client("localhost", 2500);
 *
 * @param host_n: hostname where the server is running on
 * @param port_n: port of the server
 */
Client::Client(std::string& host_n, unsigned int& port_n) :  host(host_n), port(port_n), sockfd(0), connected(false){

    server = gethostbyname(host.data());
    if (server == NULL) {
        perror("ERROR, no such host!");
        exit(1);
    }
    memset(&serv_addr, 0, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    memcpy((char*)&serv_addr.sin_addr.s_addr, (char*)server->h_addr, server->h_length);
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons((uint16_t)port);

    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
        perror("Error in opening socket!");
        exit(1);
    }

}

/**
 * Destructor: calls the stop() method
 */
Client::~Client() {
    stop();
}

/**
 * cleanly stop the client instance
 */
void Client::stop() {
    close(sockfd);
    connected = false;
}

/**
 * get the hostname of the server
 *
 * @return std::string host
 */
std::string Client::getHost() {
    return host;
}

/**
 * get the port of the server
 *
 * @return unsigned int port
 */
unsigned int Client::getPort() {
    return port;
}

/**
 * call this run method, to start the client
 */
void Client::run() {

    if (connect(sockfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0) {
        perror("Error in connecting to socket!");
        exit(1);
    }
    connected = true;

}

/**
 * processes the incoming messages to the client
 */
void Client::recv() {

    n = read(sockfd, buf, sizeof(buf));
    if (n < 0) {
        perror("ERROR reading to socket!");
    }
    std::cout << "Client received message: " << buf << std::endl;
}

/**
 * send messages to the server
 */
void Client::send() {

    std::cout << "Please enter a message: ";

    memset(buf, 0, sizeof(buf));
    fgets(buf, sizeof(buf), stdin);

    n = write(sockfd, buf, sizeof(buf));
    if (n < 0) {
        perror("ERROR writing to socket!");
    }
}

void Client::send(const std::string msg) {

    //std::cout << "Please enter a message: ";

    //memset(buf, 0, sizeof(buf));
    //fgets(buf, sizeof(buf), stdin);

    n = write(sockfd, msg.c_str(), msg.size());
    if (n < 0) {
        perror("ERROR writing to socket!");
    }
}