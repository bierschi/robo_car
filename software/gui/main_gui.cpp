//
// Created by christian on 31.08.18.
//
#include <iostream>
#include <QApplication>
#include <QPushButton>
#include "mainwindow.h"

int main(int argc, char **argv) {
    std::cout << "Test GUI" << std::endl;

    QApplication app(argc, argv);
    MainWindow w;

    w.show();

    return app.exec();
}