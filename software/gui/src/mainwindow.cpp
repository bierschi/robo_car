
#include <iostream>
#include <thread>
#include <iomanip>
#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    client(NULL),
    connected(false)
{
    ui->setupUi(this);
    this->setWindowTitle("RoboCar");

    connect(ui->exit_pb, SIGNAL(clicked()), this, SLOT(close()));
    connect(ui->connect_pb, SIGNAL(clicked()), this, SLOT(conServer()));
    connect(ui->stream_pb, SIGNAL(clicked()), this, SLOT(stream()));
    connect(ui->send_cmd_pb, SIGNAL(clicked()), this, SLOT(send_cmd()));
    connect(ui->forward_pb, SIGNAL(clicked()), this, SLOT(forward()));
    connect(ui->backward_pb, SIGNAL(clicked()), this, SLOT(backward()));
    connect(ui->right_pb, SIGNAL(clicked()), this, SLOT(right()));
    connect(ui->left_pb, SIGNAL(clicked()), this, SLOT(left()));
    connect(ui->right_camera_pb, SIGNAL(clicked()), this, SLOT(cameraLeft()));
    connect(ui->left_camera_pb, SIGNAL(clicked()), this, SLOT(cameraRight()));
    connect(ui->distance_pb, SIGNAL(clicked()), this, SLOT(distance()));
    connect(ui->stop_pb, SIGNAL(clicked()), this, SLOT(stop()));
    connect(ui->speedp_pb, SIGNAL(clicked()), this, SLOT(increaseSpeed()));
    connect(ui->speedm_pb, SIGNAL(clicked()), this, SLOT(decreaseSpeed()));

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::conServer() {

    std::string host = ui->host_le->text().toStdString();
    unsigned int port = (unsigned) ui->port_le->text().toInt();
    std::cout << host << " " << port << std::endl;

    client = new ClientSocket(host, port);
    connected = true;
}

void MainWindow::stream() {
    Commands stream_c = STREAM;

    if (connected)
        (*client)<<(stream_c);
}

void MainWindow::forward() {
    Commands forward_c = FORWARD;
    if (connected)
        (*client) << forward_c;
}

void MainWindow::backward() {
    Commands backward_c = BACKWARD;
    if (connected)
    (*client) << backward_c;
}

void MainWindow::right() {
    Commands right_c = RIGHT;
    if (connected)
        (*client) << right_c;
}

void MainWindow::left() {
    Commands left_c = LEFT;
    if (connected)
        (*client) << left_c;
}

void MainWindow::stop() {
    Commands stop_c = STOP;
    if (connected)
        (*client) << stop_c;
}

void MainWindow::increaseSpeed() {
    Commands increase_c = INCREASE_SPEED;
    if (connected)
        (*client) << increase_c;
}

void MainWindow::decreaseSpeed() {
    Commands decrease_c = DECREASE_SPEED;
    if (connected)
        (*client) << decrease_c;
}

void MainWindow::cameraRight() {
    Commands cam_r_c = CAM_R;
    if (connected)
        (*client) << cam_r_c;
}

void MainWindow::cameraLeft() {
    Commands cam_l_c = CAM_L;
    if (connected)
        (*client) << cam_l_c;
}

void MainWindow::distance() {
    Commands distance = DISTANCE;
    //std::string dist_s;

    if (connected) {

        (*client) << distance;
        //(*client) >> dist_s;
        if ( !run ){
            run = true;
            std::thread t(&MainWindow::showThread, this);
            t.detach();
        } else {
            run = false;
        }


    }
   // std::cout << "received distance: " << dist_s << std::endl;
}

void MainWindow::send_cmd() {
    std::string cmd = ui->send_cmd_le->text().toStdString();
    //client->send(cmd);
}

void MainWindow::showThread() {
    std::string dist_s;

    while (run) {
        (*client) >> dist_s;
        std::cout << "distance: " << dist_s << std::endl;
        ui->distance_lcd->display(QString::fromStdString(dist_s));
    }

}