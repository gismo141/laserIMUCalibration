#include "filter/calibrator.h"

void dip::filter::calibrator::setupICP(
  pcl::IterativeClosestPoint<PointT, PointT>* icp,
  PointCloudT::Ptr cloud_1, PointCloudT::Ptr cloud_2) {
  icp->setRANSACOutlierRejectionThreshold(1.1);
  icp->setRANSACIterations(20);
  icp->setMaxCorrespondenceDistance(5);
  icp->setMaximumIterations(1);
  icp->setTransformationEpsilon(1e-8);
  icp->setEuclideanFitnessEpsilon(1e-3);
  icp->setInputCloud(cloud_1);
  icp->setInputTarget(cloud_2);
}

void dip::filter::calibrator::toEuler(Eigen::Matrix4f matrix) {
  float x, y, z, roll, pitch, yaw;

  pcl::getTranslationAndEulerAngles(Eigen::Affine3f(matrix),
                                    x, y, z, roll, pitch, yaw);
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n", x, y, z);
  printf("Rotation vector :\n");
  printf("r = < %6.3f, %6.3f, %6.3f >\n\n", pcl::rad2deg(roll),
         pcl::rad2deg(pitch), pcl::rad2deg(yaw));
}

void dip::filter::calibrator::printTransformation(
  pcl::IterativeClosestPoint<PointT, PointT>* icp) {
  if (icp->hasConverged()) {
    std::cout << std::endl
              << "ICP converged: " << icp->getFitnessScore()
              << std::endl << std::endl;
  }
}

int32_t dip::filter::calibrator::calibrateLaserPose(char* argv[]) {
  PointCloudT::Ptr scan1(new PointCloudT);
  PointCloudT::Ptr scan2(new PointCloudT);
  PointCloudT::Ptr transformation(new PointCloudT);

  if (pcl::io::loadPCDFile<PointT>(argv[1], *scan1) == -1) {
    PCL_ERROR("Couldn't read file %s\n", argv[1]);
    return (-1);
  }

  if (pcl::io::loadPCDFile<PointT>(argv[2], *scan2) == -1) {
    PCL_ERROR("Couldn't read file %s\n", argv[2]);
    return (-1);
  }

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  setupICP(&icp, scan1, scan2);
  icp.align(*transformation);
  printTransformation(&icp);
  toEuler(icp.getFinalTransformation());

  pcl::visualization::PCLVisualizer viewer("Laser -> IMU Calibration");
  std::vector<int> viewports = {0, 0};
  gui::visualizer::setupVisualizer(&viewer, &viewports);
  gui::visualizer::setupViewerContent(scan1, scan2, transformation, &viewer,
                                      &viewports);
  viewer.spin();

  return (0);
}