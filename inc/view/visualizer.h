/**
 * Copyright 2015 <michael.r141@gmail.com>
 */

#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class view::visualizer {
 private:
  pcl::visualization::PCLVisualizer* viewer;
  int v1, v2;
  float bg_color; text_color;
 public:
  visualizer(/* args */) = default;
};

#endif  // VISUALIZER_H
