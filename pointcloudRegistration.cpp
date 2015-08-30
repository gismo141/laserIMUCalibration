/**
 * Copyright 2015 <michael.r141@gmail.com>
 *
 * @file    pointcloudRegistration.cpp
 * @author  [Michael Riedel](mailto:michael.r141@gmail.com?subject=pointcloudRegistration.cpp)
 */

#include "pointcloudRegistration.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/covariance_sampling.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/icp.h>

Eigen::Matrix4d
filter::icp(const pointCloud::Ptr source
            , const pointCloud::Ptr target
            , double* fitness_score
            , const double iterations)
{
  pointCloud::Ptr temp (new pointCloud);

  Eigen::Matrix4d transformationBetweenScans;

  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
//  icp.setRANSACOutlierRejectionThreshold(1.1);
//  icp.setRANSACIterations(50);
//  icp.setMaxCorrespondenceDistance(5);
  icp.setMaximumIterations(iterations);
//  icp.setTransformationEpsilon(1e-8);
//  icp.setEuclideanFitnessEpsilon(0.001);

  icp.setInputCloud(source);
  icp.setInputTarget(target);
  icp.align(*temp);

  transformationBetweenScans = icp.getFinalTransformation().cast<double>();
  *fitness_score = icp.getFitnessScore();
#ifdef DEBUG
  std::cout << "Score: " << icp.getFitnessScore() << std::endl;
#endif

  temp.reset();
  return transformationBetweenScans;
}

Eigen::Matrix4d
filter::icpLM(const pointCloud::Ptr source
              , const pointCloud::Ptr target
              , double* fitness_score
              , const double iterations)
{
  pointCloud::Ptr temp (new pointCloud);

  Eigen::Matrix4d transformationBetweenScans;

  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
//  icp.setRANSACOutlierRejectionThreshold(1.1);
//  icp.setRANSACIterations(50);
//  icp.setMaxCorrespondenceDistance(5);
  icp.setMaximumIterations(iterations);
//  icp.setTransformationEpsilon(1e-8);
//  icp.setEuclideanFitnessEpsilon(0.001);

  icp.setInputCloud(source);
  icp.setInputTarget(target);
  icp.align(*temp);

  transformationBetweenScans = icp.getFinalTransformation().cast<double>();
  *fitness_score = icp.getFitnessScore();
#ifdef DEBUG
  std::cout << "Score: " << icp.getFitnessScore() << std::endl;
#endif

  temp.reset();
  return transformationBetweenScans;
}

void
filter::downsample(pointCloud::Ptr cloud)
{
  pointCloud::Ptr temp (new pointCloud);
  pcl::copyPointCloud(*cloud, *temp);

  pcl::VoxelGrid<pcl::PointNormal> grid;
  grid.setLeafSize (0.1, 0.1, 0.1);
  grid.setInputCloud(temp);
  grid.filter(*cloud);

  temp.reset();
}

void
filter::bilateralFiltering(pointCloud::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *input);

  pcl::FastBilateralFilter<pcl::PointXYZ> bilateral_filter;
  bilateral_filter.setInputCloud (input);
  bilateral_filter.setSigmaS (5);
  bilateral_filter.setSigmaR (0.005f);
  bilateral_filter.filter (*output);

  pcl::copyPointCloud(*output, *cloud);

  input.reset();
  output.reset();
}

void
filter::movingLeastSquares(
  pointCloud::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal> output;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud, *input);

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  mls.setInputCloud (input);
  mls.setPolynomialFit (true);
  mls.setPolynomialOrder (3);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  mls.process(output);

  pcl::copyPointCloud(output, *cloud);

  input.reset();
  tree.reset();
}

void
filter::computeSurfaceNormals(
  pointCloud::Ptr cloud)
{
  pointCloud::Ptr temp (new pointCloud);
  pcl::copyPointCloud(*cloud, *temp);

//  pcl::IntegralImageNormalEstimation<pcl::PointNormal, pcl::PointNormal> normal_estimation_ii;
//  normal_estimation_ii.setNormalEstimationMethod (normal_estimation_ii.AVERAGE_DEPTH_CHANGE);
//  normal_estimation_ii.setMaxDepthChangeFactor (0.01f);
//  normal_estimation_ii.setDepthDependentSmoothing (true);
//  normal_estimation_ii.setNormalSmoothingSize (50.0f);
//  normal_estimation_ii.setInputCloud (temp);
//  normal_estimation_ii.compute (*cloud);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
  tree->setInputCloud (temp);
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (10);
  norm_est.setInputCloud (temp);
  norm_est.compute (*cloud);

  temp.reset();
}

void
filter::covarianceDownsampling(
  pointCloud::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp2;
  pcl::copyPointCloud(*cloud, *temp);
  pcl::PointCloud<pcl::PointNormal>::Ptr temp3 (new pcl::PointCloud<pcl::PointNormal>);

  pcl::CovarianceSampling<pcl::PointXYZ, pcl::PointNormal> covariance_sampling;
  covariance_sampling.setNumberOfSamples (temp->size () / 10);
  covariance_sampling.setInputCloud (temp);
  covariance_sampling.setNormals (temp3);
  covariance_sampling.filter (*temp2);

  pcl::copyPointCloud(*temp2, *cloud);

  temp.reset();
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr
filter::fastPointFeatureHistogram(const pointCloud::Ptr input)
{
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33>);

  pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (input);
  fpfh.setInputNormals (input);

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  fpfh.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (0.05);

  // Compute the features
  fpfh.compute (*fpfhs);

  return fpfhs;
}

Eigen::Matrix4d
filter::initialAlignment(
  pointCloud::Ptr source
  , pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_descriptors
  , pointCloud::Ptr target
  , pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_descriptors)
{
  Eigen::Matrix4d initialAligning;

  pointCloud::Ptr temp (new pointCloud);
  pcl::copyPointCloud(*source, *temp);

  pcl::SampleConsensusInitialAlignment<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> sac;
  sac.setMinSampleDistance (1);
  sac.setMaxCorrespondenceDistance (5);
  sac.setMaximumIterations (20);

  sac.setInputCloud (source);
  sac.setSourceFeatures (source_descriptors);

  sac.setInputTarget (target);
  sac.setTargetFeatures (target_descriptors);

  sac.align(*temp);
  initialAligning = sac.getFinalTransformation().cast<double>();

  temp.reset();

  return initialAligning;
}
