#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ClientSocket.h"

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
    bool connected;
    bool run=false;

private slots:
    void conServer();
    void disConServer();
    void stream();
    void forward();
    void backward();
    void straight();
    void right();
    void left();
    void stop();
    void increaseSpeed();
    void decreaseSpeed();
    void setDesiredSpeed();
    void cameraRight();
    void cameraLeft();
    void distance();
    void send_cmd();
    void showThread();
};

#endif // MAINWINDOW_H
