/**
 * Copyright 2015 <michael.r141@gmail.com>
 */

#ifndef _CALIBRATOR_H
#define _CALIBRATOR_H

#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>

#include <vector>
#include <string>

#include "gui/visualizer.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
namespace dip {
namespace filter {
class calibrator {
 private:
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  std::string scan1FN, scan2FN;
  PointCloudT::Ptr scan1;
  PointCloudT::Ptr scan2;
  PointCloudT::Ptr transformation;
 public:
  calibrator();
  calibrator(std::string scan1, std::string scan2);
  void setupICP(void);
  void printTransformation(void);
  void printTransformation(std::string path);
  void toEuler(Eigen::Matrix4f matrix);
  int32_t plotICPErrors(std::vector<std::string>* files,
                        uint32_t milliseconds);
  int32_t scanFlight(std::vector<std::string>* files);
  int32_t calibrateLaserPose(void);
};
}  // namespace filter
}  // namespace dip

#endif  // _CALIBRATOR_H
