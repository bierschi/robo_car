#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDesktopWidget>
#include <QCloseEvent>
#include "ClientSocket.h"
#include "SocketException.h"


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
    bool run;
    bool closeFlag;

private slots:
    void closeWindow();
    void closeEvent(QCloseEvent *event);
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
