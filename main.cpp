/**
 * Copyright 2015 <michael.r141@gmail.com>
 */

// #include <QApplication>

#include "filter/calibrator.h"

int main(int argc, char* argv[]) {
  if (argc < 3) {
    return 1;
  }

  return dip::filter::calibrator::calibrateLaserPose(argv);
  // QApplication app(argc, argv);
  // laserIMUCalibration mainWindow;
  // mainWindow.show();

  // return app.exec();
}
