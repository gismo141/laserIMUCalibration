#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> PCLColorT;

namespace gui {
static const float bgGray = 0.0f;  // Black
static const float txtGray = 1.0f - bgGray;
class visualizer {
 private:
 public:
  visualizer(/* args */) = default;
  static void print4x4Matrix(const Eigen::Matrix4d& matrix);
  static void setupViewerContent(PointCloudT::Ptr scan1, PointCloudT::Ptr scan2,
                                 PointCloudT::Ptr transformation,
                                 pcl::visualization::PCLVisualizer* viewer,
                                 std::vector<int>* viewports);
  static void setupVisualizer(pcl::visualization::PCLVisualizer* viewer,
                              std::vector<int>* viewports);
};
} // gui

#endif // VISUALIZER_H
