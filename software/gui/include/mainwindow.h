#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDesktopWidget>
#include <QCloseEvent>

#include "communication/ClientSocket.h"
#include "communication/SocketException.h"


/**
 * /CLASS ClientSocket
 *
 * creates a MainWindow object
 */
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


private:
    Ui::MainWindow *ui;
    ClientSocket *client;
    QPixmap noWifi, wifi;
    bool connected, closeFlag;

private slots:

    void initSlots();
    void closeWindow();

    // Qt events
    void keyPressEvent(QKeyEvent *event);
    void closeEvent(QCloseEvent *event);

    // connect/disconnect to server on RoboCar
    void conServer();
    void disConServer();

    // control the Robocar
    void forward();
    void backward();
    void straight();
    void right();
    void left();
    void stop();
    void increaseSpeed();
    void decreaseSpeed();
    void cameraRight();
    void cameraLeft();
    void saveMap();
    void resetMap();
    void stream();

    void send_cmd();
    void setDesiredSpeed();

};

#endif // MAINWINDOW_H
