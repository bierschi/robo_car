#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "client.h"

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
    Client *client;

private slots:
    void conServer();
    void stream();
    void forward();
    void backward();
    void right();
    void left();
    void send_cmd();
};

#endif // MAINWINDOW_H
