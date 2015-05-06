/**
 * Copyright 2015 <michael.r141@gmail.com>
 */

#include "view/visualizer.h"

void view::visualizer::visualizer() {
  bg_color = 0.0;  // Black
  text_color = 1.0 - bckgr_gray_level;
  viewer = new pcl::visualization::PCLVisualizer("ICP demo");
  v1(0);
  v2(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
}
