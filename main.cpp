/**
 * Copyright 2015 <michael.r141@gmail.com>
 */

#include <QApplication>

#include "view/laserIMUCalibration.h"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  laserIMUCalibration mainWindow;
  mainWindow.show();

  return app.exec();
}
