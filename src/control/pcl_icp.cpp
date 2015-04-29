/**
 * Copyright 2015 <Michael Riedel, michael.riedel@dlr.de>
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

#include <iostream>

#ifndef QUAD_LENGTH
#define QUAD_LENGTH 1.9f
#endif

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * @brief   The main-function
 * @details Shows the ICP-problem that the transformation is completely wrong,
 *          when:
 *          - the pointcloud consists of orthogonal points(formed as quader),
 *          - where the vertices between the side-planes are more than the
 *            double length of the vertice of the side-length,
 *          - and the pointcloud is moved more than the length of the smaller
 *            vertice.
 *
 *          Example:
 *          - 8 Points(quader)
 *            | No. | x   | y   | z   |
 *            | :-: | :-: | :-: | :-: |
 *            |  0  |  0  |  0  |  0  |
 *            |  1  |  2  |  0  |  0  |
 *            |  2  |  0  |  1  |  0  |
 *            |  3  |  0  |  0  |  1  |
 *            |  4  |  2  |  1  |  0  |
 *            |  5  |  0  |  0  |  1  |
 *            |  6  |  2  |  0  |  1  |
 *            |  7  |  2  |  1  |  1  |
 *          - Problem occurs when pointcloud is translated(x,y,z) =(1,0,0)
 *
 * @param   argc No. of arguments given
 * @param   argv Pointer to the shell-string
 *
 * @return  The exit-code(atm only 0 = finished execution)
 */
int
main(int argc, char** argv) {
    PointCloudT::Ptr cloud_in(new PointCloudT);
    PointCloudT::Ptr cloud_out(new PointCloudT);
    PointCloudT::Ptr Final(new PointCloudT);

    // The point-cloud data
    cloud_in->width    = 5;
    cloud_in->height   = 5;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);

    cloud_in->points[0] = PointT(0, 0, 0);
    cloud_in->points[1] = PointT(QUAD_LENGTH, 0, 0);
    cloud_in->points[2] = PointT(0, 1, 0);
    cloud_in->points[3] = PointT(0, 0, 1);
    cloud_in->points[4] = PointT(QUAD_LENGTH, 1, 0);
    cloud_in->points[5] = PointT(0, 1, 1);
    cloud_in->points[6] = PointT(QUAD_LENGTH, 0, 1);
    cloud_in->points[7] = PointT(QUAD_LENGTH, 1, 1);

    *cloud_out = *cloud_in;

    // ICP configuration
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setRANSACOutlierRejectionThreshold(1.1);
    icp.setRANSACIterations(20);
    icp.setMaxCorrespondenceDistance(5);
    icp.setMaximumIterations(20);
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
        cloud_in
        , static_cast<int>(255) * txt_gray_lvl
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
        Final, 180, 20, 20);
    viewer.addPointCloud(Final, cloud_icp_color_h, "cloud_icp_v2", v2);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_icp_v2");

    viewer.setCameraPosition(-4.0, 3.0, 5.7, 0.3, 0.9, -0.3, 0);
    viewer.setSize(720, 450);

    for (;;) {
        for (size_t i = 0; i < cloud_out->points.size(); ++i) {
            cloud_out->points[i].x = cloud_out->points[i].x + 0.02f;
        }

        icp.align(*Final);

        std::cout << "has converged: " << icp.hasConverged() << " score: " <<
                  icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;

        viewer.updatePointCloud(cloud_out, cloud_tr_color_h, "cloud_tr_v1");
        viewer.updatePointCloud(cloud_out, cloud_tr_color_h, "cloud_tr_v2");
        viewer.updatePointCloud(Final, cloud_icp_color_h, "cloud_icp_v2");

        viewer.spinOnce(100);
    }

    return (0);
}
