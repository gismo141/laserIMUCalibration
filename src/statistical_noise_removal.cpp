/**
 * Copyright 2015 <Michael Riedel, michael.riedel@dlr.de>
 */
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>
#include <string>

#ifndef NOISE
#define NOISE 1.0f
#endif

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> PCLColorT;

unsigned int seed = 12345;

int main(int argc, char* argv[]) {
    PointCloudT::Ptr cloud_in(new PointCloudT);
    PointCloudT::Ptr cloud_noised(new PointCloudT);
    PointCloudT::Ptr cloud_cleaned(new PointCloudT);

    if (pcl::io::loadPLYFile(argv[1], *cloud_in) < 0) {
        PCL_ERROR("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }

    *cloud_noised = *cloud_in;
    *cloud_cleaned = *cloud_in;

    // add noise to cloud
    for (size_t i = 0; i < cloud_noised->points.size(); ++i) {
        cloud_noised->points[i].x +=
            (NOISE * rand_r(&seed) / (RAND_MAX + 1.0f)) - (NOISE / 2);
        cloud_noised->points[i].y +=
            (NOISE * rand_r(&seed) / (RAND_MAX + 1.0f)) - (NOISE / 2);
        cloud_noised->points[i].z +=
            (NOISE * rand_r(&seed) / (RAND_MAX + 1.0f)) - (NOISE / 2);
    }

    // Clean the noise
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_noised);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.2);
    sor.filter(*cloud_cleaned);

    // visualization
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    pcl::visualization::PCLVisualizer viewer("ICP demo");
    int v1(0);
    int v2(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    PCLColorT cloud_in_color_h(cloud_in
                               , static_cast<int>(255) * txt_gray_lvl
                               , static_cast<int>(255) * txt_gray_lvl
                               , static_cast<int>(255) * txt_gray_lvl);
    PCLColorT cloud_noised_color_h(cloud_noised, 20, 180, 20);
    PCLColorT cloud_cleaned_color_h(cloud_cleaned, 180, 20, 20);

    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_v1");
    viewer.addPointCloud(cloud_noised, cloud_noised_color_h, "cloud_noised_v1"
                         , v1);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_noised_v1");

    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_v2");
    viewer.addPointCloud(cloud_noised, cloud_noised_color_h
                         , "cloud_noised_v2"
                         , v2);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_noised_v2");
    viewer.addPointCloud(cloud_cleaned, cloud_cleaned_color_h
                         , "cloud_cleaned_v2"
                         , v2);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_cleaned_v2");

    viewer.setCameraPosition(3, 3, 6, 0, 0, 0, 0);
    viewer.setSize(720, 450);

    for (;;) {
        viewer.spinOnce();
    }

    return (0);
}
