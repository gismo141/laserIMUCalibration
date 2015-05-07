/**
 * Copyright 2015 <michael.riedel@dlr.de>
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>

#include <iostream>
#include <fstream>
#include <cstring>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> PCLColorT;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        return 1;
    }

    PointCloudT::Ptr scandata1(new PointCloudT);
    PointCloudT::Ptr scandata2(new PointCloudT);
    PointCloudT::Ptr transformation(new PointCloudT);
    PointCloudT::Ptr goICP(new PointCloudT);

    if (pcl::io::loadPCDFile<PointT>(argv[1], *scandata1) == -1) {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return (-1);
    }

    if (pcl::io::loadPCDFile<PointT>(argv[2], *scandata2) == -1) {
        PCL_ERROR("Couldn't read file %s\n", argv[2]);
        return (-1);
    }

    // ICP configuration
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputCloud(scandata2);
    icp.setInputTarget(scandata1);
    icp.setRANSACOutlierRejectionThreshold(1.1);
    icp.setRANSACIterations(20);
    icp.setMaxCorrespondenceDistance(5);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-3);
    icp.setMaximumIterations(20);
    icp.align(*transformation);

    std::cout << "has converged: " << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl << std::endl;

    float x, y, z, roll, pitch, yaw;

    pcl::getTranslationAndEulerAngles(
        Eigen::Affine3f(icp.getFinalTransformation())
        , x, y, z, roll, pitch, yaw);
    std::cout << "T[x,y,z]: "
              << x << ", "
              << y << ", "
              << z << std::endl;
    std::cout << "Roll, Pitch, Yaw:"
              << pcl::rad2deg(roll) << ", "
              << pcl::rad2deg(pitch) << ", "
              << pcl::rad2deg(yaw) << std::endl;

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Bunny Transformation
    // // without downsampling (took 753 seconds)
    // // rotation (row, column)
    // transform_1(0, 0) = -0.0082047;
    // transform_1(0, 1) = 0.0022767;
    // transform_1(0, 2) = 0.9999637;
    // transform_1(1, 0) = -0.0079257;
    // transform_1(1, 1) = 0.9999658;
    // transform_1(1, 2) = -0.0023420;
    // transform_1(2, 0) = -0.9999349;
    // transform_1(2, 1) = -0.0079446;
    // transform_1(2, 2) = -0.0081864;

    // // translation
    // transform_1(0, 3) = 0.2173620;
    // transform_1(1, 3) = -0.1504636;
    // transform_1(2, 3) = 0.0738854;

    // // with downsampling (500 pts, took 3.77 seconds)
    // // rotation (row, column)
    // transform_1(0, 0) = -0.0000959;
    // transform_1(0, 1) = -0.0115725;
    // transform_1(0, 2) = 0.9999328;
    // transform_1(1, 0) = 0.0134364;
    // transform_1(1, 1) = 0.9998424;
    // transform_1(1, 2) = 0.0115722;
    // transform_1(2, 0) = -0.9999096;
    // transform_1(2, 1) = 0.0134363;
    // transform_1(2, 2) = 0.0000594;

    // // translation
    // transform_1(0, 3) = 0.2146019;
    // transform_1(1, 3) = -0.1427068;
    // transform_1(2, 3) = 0.0783229;

    // // scan transformation (500 pts, 27.127 seconds)
    // // 1412242514_182498_sc -> 1412242514_981709_sc
    // // rotation (row, column)
    // transform_1(0, 0) = 0.9998000;
    // transform_1(0, 1) = 0.0014268;
    // transform_1(0, 2) = 0.0199483;
    // transform_1(1, 0) = -0.0016409;
    // transform_1(1, 1) = 0.9999412;
    // transform_1(1, 2) = 0.0107217;
    // transform_1(2, 0) = -0.0199318;
    // transform_1(2, 1) = -0.0107523;
    // transform_1(2, 2) = 0.9997435;

    // // translation
    // transform_1(0, 3) = 0.1484375;
    // transform_1(1, 3) = 0.1796875;
    // transform_1(2, 3) = -0.3359375;

    // scan transformation
    // 1412242514_182498_sc -> 1412242514_981709_sc
    // rotation (row, column)
    transform_1(0, 0) = 0.9992837;
    transform_1(0, 1) = -0.0274583;
    transform_1(0, 2) = 0.0260414;
    transform_1(1, 0) = 0.0279404;
    transform_1(1, 1) = 0.9994415;
    transform_1(1, 2) = -0.0183327;
    transform_1(2, 0) = -0.0255235;
    transform_1(2, 1) = 0.0190471;
    transform_1(2, 2) = 0.9994928;

    // translation
    transform_1(0, 3) = 0.0703125;
    transform_1(1, 3) = 0.0859375;
    transform_1(2, 3) = -0.2421875;

    pcl::transformPointCloud(*scandata2, *goICP, transform_1);

    // visualization
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    pcl::visualization::PCLVisualizer viewer("ICP demo");
    int v1(0);
    viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);

    // white
    PCLColorT scandata1_color_h(scandata1, static_cast<int>(255) * txt_gray_lvl
                                , static_cast<int>(255) * txt_gray_lvl
                                , static_cast<int>(255) * txt_gray_lvl);
    // green
    PCLColorT scandata2_color_h(scandata2, 20, 180, 20);
    // red
    PCLColorT transformation_color_h(transformation, 180, 20, 20);
    // violet
    PCLColorT goICP_color_h(transformation, 200, 20, 180);

    viewer.addPointCloud(scandata1, scandata1_color_h, "scandata1_v1", v1);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scandata1_v1");
    viewer.addPointCloud(scandata2, scandata2_color_h, "scandata2_v1", v1);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scandata2_v1");
    viewer.addPointCloud(transformation, transformation_color_h
                         , "transformation_v1", v1);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformation_v1");
    viewer.addPointCloud(goICP, goICP_color_h
                         , "goICP_v1", v1);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "goICP_v1");

    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947
                             , -0.256907, 0);
    viewer.setSize(720, 450);

    viewer.spin();
    return 0;
}
