/**
 * Copyright 2015 <michael.r141@gmail.com>
 */

#include "filter/noiseTool.h"

// add noise to cloud
void dip::filter::noiseTool::addNoise(PointCloudT::Ptr cloud, float noise) {
  uint32_t seed = 12345;

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x +=
      (noise * rand_r(&seed) / (RAND_MAX + noise)) - (noise / 2);
    cloud->points[i].y +=
      (noise * rand_r(&seed) / (RAND_MAX + noise)) - (noise / 2);
    cloud->points[i].z +=
      (noise * rand_r(&seed) / (RAND_MAX + noise)) - (noise / 2);
  }
}

// Clean the noise
void dip::filter::noiseTool::removeNoise(
  pcl::StatisticalOutlierRemoval<PointT>* sor,
  PointCloudT::Ptr cloud_noised,
  PointCloudT::Ptr cloud_cleaned) {
  sor->setInputCloud(cloud_noised);
  sor->setMeanK(50);
  sor->setStddevMulThresh(0.2);
  sor->filter(*cloud_cleaned);
}

// move cloud
void dip::filter::noiseTool::translateCloud(PointCloudT::Ptr cloud,
    Eigen::Vector3f translationXYZ) {
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x += translationXYZ[0];
    cloud->points[i].y += translationXYZ[1];
    cloud->points[i].z += translationXYZ[2];
  }
}
