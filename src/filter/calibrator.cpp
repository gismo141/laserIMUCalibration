/**
* Copyright 2015 <michael.r141@gmail.com>
*/

#include "filter/calibrator.h"

#include <vector>
#include <string>
#include <fstream>

dip::filter::calibrator::calibrator():
  scan1(new PointCloudT),
  scan2(new PointCloudT),
  transformation(new PointCloudT) {}

dip::filter::calibrator::calibrator(std::string scan1, std::string scan2):
  scan1FN(scan1),
  scan2FN(scan2),
  scan1(new PointCloudT),
  scan2(new PointCloudT),
  transformation(new PointCloudT) {
  if (pcl::io::loadPCDFile<PointT>(scan1FN, *this->scan1) == -1) {
    PCL_ERROR("Couldn't read file %s\n", scan1FN.c_str());
  }

  if (pcl::io::loadPCDFile<PointT>(scan2FN, *this->scan2) == -1) {
    PCL_ERROR("Couldn't read file %s\n", scan2FN.c_str());
  }
}

void dip::filter::calibrator::setupICP(void) {
  icp.setRANSACOutlierRejectionThreshold(1.1);
  icp.setRANSACIterations(20);
  icp.setMaxCorrespondenceDistance(5);
  icp.setMaximumIterations(1);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-3);
  icp.setInputCloud(scan1);
  icp.setInputTarget(scan2);
}

void dip::filter::calibrator::toEuler(Eigen::Matrix4f matrix) {
  float x, y, z, phi, theta, psi;

  pcl::getTranslationAndEulerAngles(Eigen::Affine3f(matrix),
                                    x, y, z, phi, theta, psi);
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n", x, y, z);
  printf("Rotation vector :\n");
  printf("r = < %6.3f, %6.3f, %6.3f >\n\n", pcl::rad2deg(phi),
         pcl::rad2deg(theta), pcl::rad2deg(psi));
}

void dip::filter::calibrator::printTransformation(void) {
  if (icp.hasConverged()) {
    std::cout << std::endl
              << "ICP converged: " << icp.getFitnessScore()
              << std::endl << std::endl;
  }
}

std::string dip::filter::calibrator::parseTimestamp(std::string text) {
  std::string purge = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_./";
  text.erase(0, text.find_first_not_of(purge));
  text.replace(10, 1, ".");
  text.erase(17, 22);
  return text;
}

void dip::filter::calibrator::printTransformation(std::string path) {
  if (icp.hasConverged()) {
    std::ofstream outputFile;
    outputFile.open(path, std::ios_base::app);
    outputFile << parseTimestamp(scan1FN) << ";"
               << parseTimestamp(scan2FN) << ";"
               << icp.getFitnessScore() << "\n";
  }
}

int32_t dip::filter::calibrator::plotICPScore(
  std::vector<std::string>* files, uint32_t seconds) {
  uint32_t iterations = 0;

  for (std::string file : *files) {
    if (pcl::io::loadPCDFile<PointT>(file, *scan1) == -1) {
      PCL_ERROR("Couldn't read file %s\n", file.c_str());
      return (-1);
    }

    uint32_t nextScan = iterations + (seconds * 10);

    if (nextScan < (*files).size()) {
      std::string compareTo = (*files)[nextScan];

      if (pcl::io::loadPCDFile<PointT>(compareTo, *scan2) == -1) {
        PCL_ERROR("Couldn't read file %s\n", file.c_str());
        return (-1);
      }

      scan1FN = file;
      scan2FN = compareTo;
      setupICP();
      icp.align(*transformation);
      std::string filename = "icp_fitness_"
                             + std::to_string(seconds)
                             + ".csv";
      printTransformation(filename);
      std::cout << "\r" << ++iterations * 100 / (*files).size() << "%"
                << std::flush;

    } else {
      break;
    }
  }

  return (0);
}

int32_t dip::filter::calibrator::scanFlight(std::vector<std::string>* files) {
  pcl::visualization::PCLVisualizer viewer("Laser Flight");
  std::vector<int> viewports = {0};
  gui::visualizer::setupVisualizer(&viewer, &viewports);
  uint32_t firstTime = 1;

  for (std::string file : *files) {
    if (pcl::io::loadPCDFile<PointT>(file, *scan1) == -1) {
      PCL_ERROR("Couldn't read file %s\n", file.c_str());
      return (-1);
    }

    if (firstTime) {
      gui::visualizer::setupViewerContent(scan1, &viewer, &viewports);
      --firstTime;

    } else {
      viewer.updatePointCloud(scan1, "scan1V1");
    }

    viewer.spinOnce();
    std::cout << "\r" << parseTimestamp(file) << std::flush;
  }

  return (0);
}

int32_t dip::filter::calibrator::calibrateLaserPose(void) {
  pcl::visualization::PCLVisualizer viewer("Laser -> IMU Calibration");
  std::vector<int> viewports = {0, 0};
  gui::visualizer::setupVisualizer(&viewer, &viewports);

  setupICP();
  icp.align(*transformation);
  printTransformation();
  toEuler(icp.getFinalTransformation());

  gui::visualizer::setupViewerContent(scan1, scan2, transformation, &viewer,
                                      &viewports);
  viewer.spinOnce();
  return (0);
}
