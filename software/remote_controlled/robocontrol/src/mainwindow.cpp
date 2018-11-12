
#include <iostream>
#include <thread>
#include <iomanip>
#include <QDebug>
#include <QtWidgets/QMessageBox>

#include "mainwindow.h"
#include "ui_mainwindow.h"


/**
 * Constructor for a MainWindow instance
 *
 * USAGE:
 *      MainWindow w;
 *
 * @param parent: QWidget*
 */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow),
                                          client(NULL),
                                          connected(false),
                                          closeFlag(true),
                                          streamMapFlag(false)
{
    // setup GUI
    ui->setupUi(this);
    this->setWindowTitle("RoboCar");
    this->setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, this->size(), qApp->desktop()->availableGeometry()));

    // init slots
    initSlots();

    ui->desired_speed_le->setToolTip("Value between -480 and 480");
    ui->disconnect_pb->setEnabled(false);
    noWifi = QPixmap("/home/christian/projects/robo_car/software/robocontrol/images/no_wifi.png");
    wifi = QPixmap("/home/christian/projects/robo_car/software/robocontrol/images/wifi.png");
    ui->wifi_l->setPixmap(noWifi);

    //only for testing purposes
    ui->host_le->setText("192.168.178.39");
    ui->port_le->setText("2501");


}

/**
 * initialize all slots
 */
void MainWindow::initSlots() {

    connect(ui->exit_pb, SIGNAL(clicked()), this, SLOT(closeWindow()));
    connect(ui->connect_pb, SIGNAL(clicked()), this, SLOT(conServer()));
    connect(ui->disconnect_pb, SIGNAL(clicked()), this, SLOT(disConServer()));

    connect(ui->forward_pb, SIGNAL(clicked()), this, SLOT(forward()));
    connect(ui->backward_pb, SIGNAL(clicked()), this, SLOT(backward()));
    connect(ui->straight_pb, SIGNAL(clicked()), this, SLOT(straight()));
    connect(ui->right_pb, SIGNAL(clicked()), this, SLOT(right()));
    connect(ui->left_pb, SIGNAL(clicked()), this, SLOT(left()));
    connect(ui->stop_pb, SIGNAL(clicked()), this, SLOT(stop()));
    connect(ui->speedp_pb, SIGNAL(clicked()), this, SLOT(increaseSpeed()));
    connect(ui->speedm_pb, SIGNAL(clicked()), this, SLOT(decreaseSpeed()));
    connect(ui->right_camera_pb, SIGNAL(clicked()), this, SLOT(cameraRight()));
    connect(ui->left_camera_pb, SIGNAL(clicked()), this, SLOT(cameraLeft()));
    connect(ui->start_stream_pb, SIGNAL(clicked()), this, SLOT(startStreamMap()));
    connect(ui->stop_stream_pb, SIGNAL(clicked()), this, SLOT(stopStreamMap()));
    connect(ui->save_pgm_pb, SIGNAL(clicked()), this, SLOT(saveMap()));
    connect(ui->reset_map_pb, SIGNAL(clicked()), this, SLOT(resetMap()));
    connect(ui->stream_pb, SIGNAL(clicked()), this, SLOT(stream()));

    connect(ui->apply_pb, SIGNAL(clicked()), this, SLOT(setDesiredSpeed()));
    connect(ui->send_cmd_pb, SIGNAL(clicked()), this, SLOT(send_cmd()));

}


/**
 * Destructor in MainWindow
 */
MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * close the main window
 */
void MainWindow::closeWindow() {

    QMessageBox::StandardButton resBtn = QMessageBox::question(this, "RoboCar", tr("Are you sure to quit?"));

    if (resBtn != QMessageBox::Yes) {

    } else {

        exit(0);

    }
}

/**
 * controls the key event from keyboard
 *
 * @param event: QKeyEvent*
 */
void MainWindow::keyPressEvent(QKeyEvent *event) {

    QMainWindow::keyPressEvent(event);

    if (event->key() == Qt::Key_W) {
        std::cout << "Key forward" << std::endl;
        forward();
    }
    else if (event->key() == Qt::Key_E) {
        std::cout << "Key backward" << std::endl;
        backward();
    }
    else if (event->key() == Qt::Key_A) {
        std::cout << "Key left" << std::endl;
        left();
    }
    else if (event->key() == Qt::Key_S) {
        std::cout << "Key straight" << std::endl;
        straight();
    }
    else if (event->key() == Qt::Key_D) {
        std::cout << "Key Right" << std::endl;
        right();
    }
    else if (event->key() == Qt::Key_Q) {
        std::cout << "Key stop" << std::endl;
        stop();
    }
}

/**
 * close the main window
 *
 * @param event: QCloseEvent*
 */
void MainWindow::closeEvent(QCloseEvent *event) {

    QMessageBox::StandardButton resBtn = QMessageBox::question(this, "RoboCar", tr("Are you sure to quit?"));

    if (resBtn != QMessageBox::Yes) {

        event->ignore();

    } else {
        closeFlag = false;
        disConServer();
        event->accept();

    }
}

/**
 * connect to the server on RoboCar
 */
void MainWindow::conServer() {

    std::string host = ui->host_le->text().toStdString();
    unsigned int port = (unsigned) ui->port_le->text().toInt();
    std::cout << "Set up connection ..." << std::endl;

    try {

        client = new ClientSocket(host, port);

        QMessageBox::information(this, "Information", "Connection established!");

        connected = true;
        if (connected) {

            ui->wifi_l->setPixmap(wifi);

        }

        ui->connect_pb->setEnabled(false);
        ui->disconnect_pb->setEnabled(true);

    } catch ( SocketException& e ) {

        std::cout << "SocketException caught with error message: " << e.description() << std::endl;
        QMessageBox::warning(this, "Error", "Connection failed!");

        connected = false;
    }
}

/**
 * disconnect from the server on RoboCar
 */
void MainWindow::disConServer() {

    if (connected) {

        if (closeFlag)
            QMessageBox::information(this, "Information", "Connection closed!");

        ui->disconnect_pb->setEnabled(false);
        ui->connect_pb->setEnabled(true);
        ui->wifi_l->setPixmap(noWifi);
        connected = false;
        client->closeConnection();

    }
}


// control the RoboCar with this methods

/**
 * moves the robocar forward
 */
void MainWindow::forward() {
    Commands forward_c = FORWARD;
    if (connected)
        client->sending(forward_c);
}

/**
 * moves the robocar backward
 */
void MainWindow::backward() {
    Commands backward_c = BACKWARD;
    if (connected)
        client->sending(backward_c);
}

/**
 * moves the steering servo straight ahead
 */
void MainWindow::straight() {
    Commands straight_c = STRAIGHT;
    if (connected)
        client->sending(straight_c);
}

/**
 * moves the steering servo to the right
 */
void MainWindow::right() {
    Commands right_c = RIGHT;
    if (connected)
        client->sending(right_c);
}

/**
 * moves the steering servo to the left
 */
void MainWindow::left() {
    Commands left_c = LEFT;
    if (connected)
        client->sending(left_c);
}

/**
 * stops the robocar
 */
void MainWindow::stop() {
    Commands stop_c = STOP;
    if (connected)
        client->sending(stop_c);
}

/**
 *
 */
void MainWindow::increaseSpeed() {
    Commands increase_c = INCREASE_SPEED;
    if (connected)
        client->sending(increase_c);
}

/**
 *
 */
void MainWindow::decreaseSpeed() {
    Commands decrease_c = DECREASE_SPEED;
    if (connected)
        client->sending(decrease_c);
}

/**
 * moves the camera servo to the right
 */
void MainWindow::cameraRight() {
    Commands cam_r_c = CAM_R;
    if (connected)
        client->sending(cam_r_c);
}

/**
 * moves the camera servo to the left
 */
void MainWindow::cameraLeft() {
    Commands cam_l_c = CAM_L;
    if (connected)
        client->sending(cam_l_c);
}

void MainWindow::startStreamMap() {
    Commands start_stream_map = START_STREAM_MAP;

    if (connected) {

        client->sending(start_stream_map);
        streamMapFlag = true;
        std::thread r(&MainWindow::run, this);
        r.detach();

    }
}

void MainWindow::stopStreamMap() {
    Commands stop_stream_map = STOP_STREAM_MAP;

    if (connected) {
        streamMapFlag = false;
        client->sending(stop_stream_map);
    }

}

/**
 * saves a slam map as a pgm file
 */
void MainWindow::saveMap() {
    Commands save_map = SAVE_MAP;
    if (connected)
        client->sending(save_map);
}

/**
 * resets the current slam map
 */
void MainWindow::resetMap() {
    Commands reset_map = RESET_MAP;
    if (connected)
        client->sending(reset_map);
}
/**
 * stream from camera
 */
void MainWindow::stream() {
    Commands stream_c = STREAM;

    if (connected)
        client->sending(stream_c);
}





void MainWindow::send_cmd() {
    std::string cmd = ui->send_cmd_le->text().toStdString();
    //client->send(cmd);
}
void MainWindow::setDesiredSpeed() {

    int speed = ui->desired_speed_le->text().toInt();
    std::cout << "speed: " << speed << std::endl;
    //if (connected)
    //    (*client) << decrease_c;
}

void MainWindow::run() {

    while (streamMapFlag) {

        if (connected) {
            std::vector<int> v;
            client->receiving(v);
            createTxtMapFile("array.txt", v);

        }
        std::cout << "Test" << std::endl;
        sleep(2);

    }
}

void MainWindow::createTxtMapFile(std::string fileName, std::vector<int> mapData) {

    FILE* out = fopen(fileName.c_str(), "w");
    for(int s = 0; s < mapData.size(); s++) {
        fprintf(out, "%d ", mapData[s]);

        if (s && s%400 == 0) {

            fprintf(out, "\n");

        }
    }
    fclose(out);
}