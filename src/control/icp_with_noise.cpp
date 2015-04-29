/**
 * Copyright 2015 <michael.riedel@dlr.de>
 */
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

#include <iostream>
#include <string>

#ifndef NOISE
#define NOISE 1.0f
#endif

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char* argv[]) {
    PointCloudT::Ptr cloud_in(new PointCloudT);
    PointCloudT::Ptr cloud_out(new PointCloudT);
    PointCloudT::Ptr cloud_icp(new PointCloudT);

    uint32_t ITERATIONS = 1;
    uint32_t seed = 12345;

    if (pcl::io::loadPLYFile(argv[1], *cloud_in) < 0) {
        PCL_ERROR("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }

    *cloud_out = *cloud_in;
    *cloud_icp = *cloud_in;

    // add noise to cloud
    for (size_t i = 0; i < cloud_out->points.size(); ++i) {
        cloud_out->points[i].x +=
            (NOISE * rand_r(&seed) / (RAND_MAX + 1.0f)) - (NOISE / 2);
        cloud_out->points[i].y +=
            (NOISE * rand_r(&seed) / (RAND_MAX + 1.0f)) - (NOISE / 2);
        cloud_out->points[i].z +=
            (NOISE * rand_r(&seed) / (RAND_MAX + 1.0f)) - (NOISE / 2);
    }

    // move cloud
    for (size_t i = 0; i < cloud_out->points.size(); ++i) {
        cloud_out->points[i].x = cloud_out->points[i].x + 0.9f;
    }

    // ICP configuration
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setRANSACOutlierRejectionThreshold(1.1);
    icp.setRANSACIterations(20);
    icp.setMaxCorrespondenceDistance(5);
    icp.setMaximumIterations(ITERATIONS);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-3);

    // visualization
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    pcl::visualization::PCLVisualizer viewer("ICP demo");
    int v1(0);
    int v2(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(
        cloud_in, static_cast<int>(255) * txt_gray_lvl
        , static_cast<int>(255) * txt_gray_lvl
        , static_cast<int>(255) * txt_gray_lvl);
    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_v1");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(
        cloud_out, 20, 180, 20);
    viewer.addPointCloud(cloud_out, cloud_tr_color_h, "cloud_tr_v1", v1);
    viewer.addPointCloud(cloud_out, cloud_tr_color_h, "cloud_tr_v2", v2);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_tr_v1");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_tr_v2");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(
        cloud_icp, 180, 20, 20);
    viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_icp_v2");

    viewer.setCameraPosition(3, 3, 6, 0, 0, 0, 0);
    viewer.setSize(720, 450);

    for (;;) {
        icp.setMaximumIterations(++ITERATIONS);
        *cloud_in = *cloud_icp;
        icp.align(*cloud_icp);
        viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");

        std::cout << "has converged: " << icp.hasConverged() << " score: " <<
                  icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;

        viewer.spinOnce(50);
    }

    return (0);
}
