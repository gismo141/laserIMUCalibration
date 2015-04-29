/**
 * Copyright 2015 <michael.r141@gmail.com>
 */

#include "view/laserIMUCalibration.h"
#include <QApplication>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    laserIMUCalibration mainWindow;
    mainWindow.show();

    return app.exec();
}