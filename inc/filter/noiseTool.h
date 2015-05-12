/**
 * Copyright 2015 <michael.r141@gmail.com>
 */
#ifndef NOISE_TOOL_H
#define NOISE_TOOL_H

#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace dip {
namespace filter {
class noiseTool {
 private:
 public:
  noiseTool(/* args */) = default;
  static void addNoise(PointCloudT::Ptr cloud, float noise);
  static void removeNoise(pcl::StatisticalOutlierRemoval<PointT>* sor,
                          PointCloudT::Ptr cloud_noised,
                          PointCloudT::Ptr cloud_cleaned);
  static void translateCloud(PointCloudT::Ptr cloud,
                             Eigen::Vector3f translationXYZ);
};
}  // namespace filter
}  // namespace dip


#endif  // NOISE_TOOL_H
