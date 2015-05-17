/**
 * Copyright 2015 <michael.r141@gmail.com>
 */

#include <dirent.h>
#include <vector>
#include <string>
#include <iostream>

#include "filter/calibrator.h"

std::vector<std::string> open(std::string path = ".") {
  DIR* dir;
  dirent* pdir;
  std::vector<std::string> files;

  dir = opendir(path.c_str());

  while (pdir = readdir(dir)) {
    files.push_back(path + pdir->d_name);
  }

  files.erase(std::find(files.begin(), files.end(), path + "."));
  files.erase(std::find(files.begin(), files.end(), path + ".."));

  return files;
}

int main(int argc, char* argv[]) {
  if (argc < 3) {
    return 1;
  }

  std::vector<std::string> files = open(argv[2]);
  dip::filter::calibrator calib;
  return calib.scanFlight(&files);
  // return calib.plotICPErrors(&files, atoi(argv[1]));

  // return dip::filter::calibrator::calibrateLaserPose();
}
