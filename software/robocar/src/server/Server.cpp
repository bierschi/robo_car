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

    ros::NodeHandle n;
    resetMap = n.advertise<std_msgs::String>("syscommand", 2);

    cmd;

    std::cout << "Server is being set up on port: " << port_ << std::endl;

    startThread();
}

/**
 * Destructor in Server
 */
Server::~Server() {

    delete serverSocket, sock;

}

/**
 * starts the run thread
 */
void Server::startThread() {

    std::thread server(&Server::run, this);
    server.detach();

}

/**
 * listening for a new client connection
 */
void Server::waitForClient() {

    std::cout << "Listening for new client ..." << std::endl;
    serverSocket->accept(*sock);

}

/**
 * Server run thread to handle incoming commands
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
            std::cout << "Move camera right!" << std::endl;
            car_.turnCameraRight();
            break;

        case CAM_L:
            std::cout << "Move camera left!" << std::endl;
            car_.turnCameraLeft();
            break;

        //SlamMap
        case SAVE_MAP:
            std::cout << "Save SLAM Map" << std::endl;
            system("rostopic pub /syscommand std_msgs/String 'reset'");
            //car_.saveSlamMap();
            break;

        case RESET_MAP: {

            std::cout << "Reset SLAM Map" << std::endl;
            std::thread reset(&server::reset, this);
            reset.detach();
        }
            break;

        case DISTANCE:
            std::cout << "Query current distance!" << std::endl;
            break;

            //starts the stream of the camera
        case STREAM: {
            std::cout << "Stream object!" << std::endl;
            car_.saveSlamMap();

        }
            break;

        default:
            std::cout << "Default in method Server::actions!" << std::endl;
    }
}


void Server::reset() {
    std_msgs::String msg;
    msg.data = "reset";
    resetMap.publish(msg);
}