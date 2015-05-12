#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>

#include "gui/visualizer.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
namespace dip {
namespace filter {
class calibrator {
 private:
 public:
  calibrator(/* args */) = default;
  static void setupICP(pcl::IterativeClosestPoint<PointT, PointT>* icp,
                       PointCloudT::Ptr cloud_1, PointCloudT::Ptr cloud_2);
  static void printTransformation(pcl::IterativeClosestPoint<PointT, PointT>*
                                  icp);
  static void toEuler(Eigen::Matrix4f matrix);
  static int32_t calibrateLaserPose(char* argv[]);
};
} // filter
} // dip

#endif // CALIBRATOR_H
