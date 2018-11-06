//
// Created by christian on 02.11.18.
//

#include "server/Server.h"

/**
 * Constructor for a Server instance
 *
 * USAGE:
 *      Server* server = new Server(2501, *car);
 *
 * @param port
 */
Server::Server(unsigned int port, Car& car) : port_(port), car_(car) {

    serverSocket = new ServerSocket(port);
    sock = new ServerSocket();
    cmd;

    std::cout << "Server is being set up on port: " << port_ << std::endl;

}

/**
 * Destructor in Server
 */
Server::~Server() {

    delete serverSocket, sock;

}

/**
 *
 */
void Server::waitForClient() {

    std::cout << "Listening for new client ..." << std::endl;
    serverSocket->accept(*sock);

}

/**
 *
 */
void Server::run() {

    waitForClient();

    try {

            while (true) {
                // receiving command from client
                sock->receiving(cmd);
                // execute this command
                actions(cmd);

            }

        } catch (SocketException& e) {

            std::cout << "SocketException was caught: " << e.description() << std::endl;
            run();
        }
}


/**
 * selects a appropriate action, depending on the incoming command
 *
 * @param cmd: Commands reference to execute defined commands
 */
void Server::actions(Commands& cmd) {

    switch(cmd) {

        //move the vehicle
        case FORWARD:
            std::cout << "Drive Forward!" << std::endl;
            car_.driveForward(40);

            break;

        case BACKWARD:
            std::cout << "Drive Backward!" << std::endl;
            car_.driveBackward(40);
            break;

        case STRAIGHT:
            std::cout << "Drive Straight Ahead!" << std::endl;
            car_.driveStraight();
            break;

        case RIGHT:
            std::cout << "Drive Right!" << std::endl;
            car_.driveRight();
            break;

        case LEFT:
            std::cout << "Drive Left!" << std::endl;
            car_.driveLeft();
            break;

        case STOP:
            std::cout << "Stop vehicle!" << std::endl;
            car_.driveStop();
            break;

        case INCREASE_SPEED:
            std::cout << "Increase current Speed!" << std::endl;

            break;

        case DECREASE_SPEED:
            std::cout << "Decrease current Speed!" << std::endl;

            break;

            //move the camera
        case CAM_R:
            std::cout << "Move camera right!11" << std::endl;
            car_.turnCameraRight();
            break;

        case CAM_L:
            std::cout << "Move camera left!111" << std::endl;
            car_.turnCameraLeft();
            break;

        case DISTANCE:
            std::cout << "Query current distance!" << std::endl;
            break;

            //starts the stream of the camera
        case STREAM: {
            std::cout << "Stream object!" << std::endl;

        }
            break;

        default:
            std::cout << "Default in method Server::actions!" << std::endl;
    }
}
