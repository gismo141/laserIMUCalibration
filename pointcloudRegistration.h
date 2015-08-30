/**
 * @file    pointcloudRegistration.h
 * @brief   This file implements all the filter and registration methods for
 *          point-cloud data using wrappers for the pcl.
 *
 * Copyright 2015 <michael.r141@gmail.com>
 *
 * @author  [Michael Riedel](mailto:michael.r141@gmail.com?subject=pointcloudRegistration.h)
 *
 **/

#ifndef POINTCLOUD_REGISTRATION_H
#define POINTCLOUD_REGISTRATION_H

#include <pcl/common/common_headers.h>

/**
 * @brief Typedef for shorter call of point-cloud-pointer-type
 */
typedef pcl::PointCloud<pcl::PointNormal> pointCloud;

namespace filter
{
/**
 * @brief Calls the pcl iterativeClosestPoint-algorithm
 * @details SETTINGS NEED TO BE OPTIMISED FOR OUR LASERSCANS!!!
 *
 * @param source the source point-cloud (we use last)
 * @param target the target point-cloud (we use latest)
 * @param fitness_score the pointer where to store the error-metric
 * @param iterations Number of iterations to transform
 * @return The transformation between last and latest to register them
 */
Eigen::Matrix4d icp(
    const pointCloud::Ptr source
    , const pointCloud::Ptr target
    , double* fitness_score
    , const double iterations);

/**
 * @brief Calls the pcl iterativeClosestPoint-algorithm levenberg-marquardt alternative (internally)
 * @details SETTINGS NEED TO BE OPTIMISED FOR OUR LASERSCANS!!!
 *
 * @param source the source point-cloud (we use last)
 * @param target the target point-cloud (we use latest)
 * @param fitness_score the pointer where to store the error-metric
 * @param iterations Number of iterations to transform
 * @return The transformation between last and latest to register them
 */
Eigen::Matrix4d icpLM(
    const pointCloud::Ptr source
    , const pointCloud::Ptr target
    , double* fitness_score
    , const double iterations);

/**
 * @brief Simple Voxel-Grid downsampling
 *
 * @param[inout] cloud point-cloud to sample
 */
void downsample(pointCloud::Ptr cloud);

/**
 * @brief Use bilateral filtering
 *
 * @param[inout] organized_cloud to sample
 */
void bilateralFiltering(pointCloud::Ptr organized_cloud);

/**
 * @brief Use moving least squares
 * @details NOT USED AT THE MOMENT!
 *
 * @param[inout] cloud to sample
 */
void movingLeastSquares(pointCloud::Ptr cloud);

/**
 * @brief Compute the surface normals
 *
 * @param[inout] cloud to compute on
 */
void computeSurfaceNormals(pointCloud::Ptr cloud);

/**
 * @brief Use covariances for downsampling
 *
 * @param[inout] cloud to sample
 */
void covarianceDownsampling(pointCloud::Ptr cloud);

/**
 * @brief Extracts all (F)PFHs
 *
 * @param input The point-cloud to analyse
 * @return The feature-descriptor
 */
pcl::PointCloud<pcl::FPFHSignature33>::Ptr
fastPointFeatureHistogram(const pointCloud::Ptr input);

/**
 * @brief Aligns the point-clouds initially using Sample Consensus
 * @details NOT USED AT THE MOMENT!
 *
 * @param source The first point-cloud
 * @param source_descriptors The descriptors for the first point-cloud
 * @param target The second point-cloud
 * @param target_descriptors The descriptors for the second point-cloud
 *
 * @return The transformation extracted from initial alignment
 */
Eigen::Matrix4d
initialAlignment(
    pointCloud::Ptr source
    , pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_descriptors
    , pointCloud::Ptr target
    , pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_descriptors);
}

#endif // POINTCLOUD_REGISTRATION_H
