#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

namespace dip {
namespace object {
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> PCLColorT;
} // object
} // dip

#endif // DATA_TYPES_H
