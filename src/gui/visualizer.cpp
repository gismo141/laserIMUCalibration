/**
 * Copyright 2015 <michael.r141@gmail.com>
 */

#include "gui/visualizer.h"
#include <vector>

void gui::visualizer::print4x4Matrix(const Eigen::Matrix4d& matrix) {
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1)
         , matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1)
         , matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1)
         , matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3)
         , matrix(2, 3));
}

void gui::visualizer::setupVisualizer(pcl::visualization::PCLVisualizer* viewer,
                                      std::vector<int>* viewports) {
  uint32_t viewportsAdded = 0;
  float viewportWidth = 1.0 / viewports->size();

  for (std::vector<int>::iterator it = viewports->begin();
       it != viewports->end(); ++it) {
    float startX = viewportsAdded * viewportWidth;
    float startY = 0.0;
    float endX = viewportWidth + viewportsAdded * viewportWidth;
    float endY = 1.0;

    viewer->createViewPort(startX, startY, endX, endY, *it);
    viewer->setBackgroundColor(bgGray + 0.1 * viewportsAdded,
                               bgGray + 0.1 * viewportsAdded,
                               bgGray + 0.1 * viewportsAdded, *it);
    ++viewportsAdded;
  }

  viewer->setCameraPosition(3, 3, 6, 0, 0, 0, 0);
  viewer->setSize(1024, 640);
}

void gui::visualizer::setupViewerContent(PointCloudT::Ptr scan1,
    PointCloudT::Ptr scan2,
    PointCloudT::Ptr transformation,
    pcl::visualization::PCLVisualizer* viewer,
    std::vector<int>* viewports) {
  PCLColorT white(scan1, static_cast<int>(255) * txtGray
                  , static_cast<int>(255) * txtGray
                  , static_cast<int>(255) * txtGray);
  PCLColorT green(scan2, 20, 180, 20);
  PCLColorT red(transformation, 180, 20, 20);

  viewer->addText(
    "White: \tScan 1\nGreen: \tScan 2", 10, 15, "icp_info_1", (*viewports)[0]);

  viewer->addPointCloud(scan1, white, "scan1V1", (*viewports)[0]);
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scan1V1");

  viewer->addPointCloud(scan2, green, "scan2V1", (*viewports)[0]);
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scan2V1");

  viewer->addText(
    "\nRed: \tTransformed Scan1\nGreen: \tScan 2", 10, 15, "icp_info_2",
    (*viewports)[1]);

  viewer->addPointCloud(transformation, red, "transformation", (*viewports)[1]);
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformation");

  viewer->addPointCloud(scan2, green, "scan2V2", (*viewports)[1]);
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scan2V2");
}
