
#include <iostream>
#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    client(NULL)
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
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::conServer() {

    std::string host = ui->host_le->text().toStdString();
    unsigned int port = (unsigned) ui->port_le->text().toInt();
    std::cout << host << " " << port << std::endl;
    client = new Client(host, port);
    client->run();

}

void MainWindow::stream() {
    std::string stream = "stream";
    client->send(stream);
}

void MainWindow::forward() {
    std::string forward = "forward";
    client->send(forward);
}

void MainWindow::backward() {
    std::string backward = "backward";
    client->send(backward);
}

void MainWindow::right() {
    std::string right = "right";
    client->send(right);
}

void MainWindow::left() {
    std::string left = "left";
    client->send(left);
}

void MainWindow::send_cmd() {
    std::string cmd = ui->send_cmd_le->text().toStdString();
    client->send(cmd);
}