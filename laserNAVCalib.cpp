/**
 * @file    laserNAVCalib.cpp
 * @brief   This file introduces the class LaserNAVCalibration which is
 *          used to calibrate the laser-pose with nav-data.
 *
 * Copyright 2015 <michael.r141@gmail.com>
 *
 * @author  [Michael Riedel](mailto:michael.r141@gmail.com?subject=laserNAVCalib.cpp)
 *
 **/

#include "laserNAVCalib.h"

// copy pointclouds
#include <pcl/common/io.h>
// transform pointclouds
#include <pcl/common/transforms.h>

// setw()
#include <iomanip>
// fixed
#include <ios>
// File access
#include <fstream>
#include <sstream>
#include <sys/stat.h>
// Bind function-pointer (needed for Levenberg-Marquardt)
#include <boost/bind.hpp>

// register pointclouds with different optimizations
#include "pointcloudRegistration.h"
// optimization with Levenberg-Marquardt
#include "lmmin.h"

unsigned int iterations;

unsigned int filter::LaserNAVCalibration::scans_between_last_and_latest = 10;
// used pointclouds
pointCloud::Ptr filter::LaserNAVCalibration::last_to_draw (new pointCloud);
pointCloud::Ptr filter::LaserNAVCalibration::latest_to_draw (new pointCloud);
// Average error for LM
double filter::LaserNAVCalibration::avr_cal_err = 0.0;
// optimizations
bool filter::LaserNAVCalibration::downsampling = false;
bool filter::LaserNAVCalibration::bilateral_filtering = false;
bool filter::LaserNAVCalibration::calculate_normals = false;
bool filter::LaserNAVCalibration::covariance_sampling = false;
bool filter::LaserNAVCalibration::initial_aligning = false;
filter::LaserNAVCalibration::e_registrationMethod filter::LaserNAVCalibration::ICPMethod = filter::LaserNAVCalibration::E_LEVENBERG_MARQUARDT;
size_t filter::LaserNAVCalibration::worst_pair = 0;
double filter::LaserNAVCalibration::worst_score = 0.0;

filter::LaserNAVCalibration::LaserNAVCalibration(
  double pos_x
  , double pos_y
  , double pos_z
  , double phi
  , double theta
  , double psi)
  : sampled_scans(0)
    // we have no scans received initially
  , scan_counter(0)
  , plot_all_movements(true)
  , calibration_wanted(true)
  , last(new pointCloud)
  , latest(new pointCloud)
  , nr_scan_pairs(0)
    // control flow
  , add_to_data_set(false)
  , init(true)
{
  // From the user estimated/measured pose
  filter::LaserNAVCalibration::pos_x = pos_x;
  filter::LaserNAVCalibration::pos_y = pos_y;
  filter::LaserNAVCalibration::pos_z = pos_z;
  filter::LaserNAVCalibration::phi = phi;
  filter::LaserNAVCalibration::theta = theta;
  filter::LaserNAVCalibration::psi = psi;
}

filter::LaserNAVCalibration::~LaserNAVCalibration(void)
{
  last.reset();
  latest.reset();
  last_to_draw.reset();
  latest_to_draw.reset();
  data_set.clear();
}

void
filter::LaserNAVCalibration::print(
  double pos_x, double pos_y, double pos_z
  , double phi, double theta, double psi
  , std::string name)
{
  const double width = 20;
  std::cout << std::endl
            << std::setw( width ) << std::left << name << ":" << std::endl
            << std::fixed
            << std::setw( width ) << std::right << "x" << " " << pos_x << std::endl
            << std::setw( width ) << std::right << "y" << " " << pos_y << std::endl
            << std::setw( width ) << std::right << "z" << " " << pos_z << std::endl
            << std::setw( width ) << std::right << "phi" << " " << math::c_RAD_TO_DEG * phi << std::endl
            << std::setw( width ) << std::right << "theta" << " " << math::c_RAD_TO_DEG * theta << std::endl
            << std::setw( width ) << std::right << "psi" << " " << math::c_RAD_TO_DEG * psi << std::endl
            << std::endl;
}

void
filter::LaserNAVCalibration::print(
  Eigen::Affine3d* transformation_to_print
  , std::string name)
{
  print(&transformation_to_print->matrix(), name);
}

void
filter::LaserNAVCalibration::print(
  Eigen::Matrix4d* transformation_to_print
  , std::string name)
{
  double d_x, d_y, d_z, phi, theta, psi;
  pcl::getTranslationAndEulerAngles(
    Eigen::Affine3d(*transformation_to_print), d_x, d_y, d_z, phi, theta, psi);
  print(d_x, d_y, d_z, phi, theta, psi, name);
}

bool
filter::LaserNAVCalibration::exists(
  const std::string& name)
{
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

void
filter::LaserNAVCalibration::plot(
  std::string header
  , std::string data
  , std::string filename)
{
  std::string folder = "../log/plots/";
  std::string extension = ".csv";

  std::stringstream plot_file_ss;
  plot_file_ss << folder << filename << extension;
  std::string plot_file_s = plot_file_ss.str();

  std::fstream plot_file;

  if (exists(plot_file_s)) {
    plot_file.open(
      plot_file_s.c_str()
      , std::fstream::in | std::fstream::out | std::fstream::app);
  } else {
    plot_file.open(
      plot_file_s.c_str()

      , std::fstream::in | std::fstream::out | std::fstream::app);
    plot_file << header << std::endl;
  }
  plot_file << data << std::endl;

  plot_file.close();
}

Eigen::Matrix4d
filter::LaserNAVCalibration::getMovementFrom(
  pointCloud::Ptr last
  , pointCloud::Ptr latest)
{
  Eigen::Affine3d T_movement;
  // const sensors::NavData* lastNav
  //   = boost::static_pointer_cast<pointCloud>(last)->nav_data;
  // const sensors::NavData* latestNav
  //   = boost::static_pointer_cast<pointCloud>(latest)->nav_data;

  // if (lastNav && latestNav) {
  //   double d_x, d_y, d_z, d_phi, d_theta, d_psi;

  //   d_x     = latestNav->getPos().getX()   - lastNav->getPos().getX();
  //   d_y     = latestNav->getPos().getY()   - lastNav->getPos().getY();
  //   d_z     = latestNav->getPos().getZ()   - lastNav->getPos().getZ();
  //   d_phi   = latestNav->getEuler().getX() - lastNav->getEuler().getX();
  //   d_theta = latestNav->getEuler().getY() - lastNav->getEuler().getY();
  //   d_psi   = latestNav->getEuler().getZ() - lastNav->getEuler().getZ();

  //   pcl::getTransformation(d_x, d_y, d_z, d_phi, d_theta, d_psi, T_movement);

  //   std::string movement_header = "x_1;y_1;z_1;phi_1;theta_1;psi_1;x_2;y_2;z_2;phi_2;theta_2;psi_2;d_x;d_y;d_z;d_phi;d_theta;d_psi";
  //   std::stringstream movement;
  //   movement << std::setprecision(10)
  //            << lastNav->getPos().getX() << ";"
  //            << lastNav->getPos().getY() << ";"
  //            << lastNav->getPos().getZ() << ";"
  //            << lastNav->getEuler().getX() << ";"
  //            << lastNav->getEuler().getY() << ";"
  //            << lastNav->getEuler().getZ() << ";"
  //            << latestNav->getPos().getX() << ";"
  //            << latestNav->getPos().getY() << ";"
  //            << latestNav->getPos().getZ() << ";"
  //            << latestNav->getEuler().getX() << ";"
  //            << latestNav->getEuler().getY() << ";"
  //            << latestNav->getEuler().getZ() << ";"
  //            << d_x << ";" << d_y << ";" << d_z << ";" << d_phi << ";" << d_theta << ";" << d_psi;
  //   plot(movement_header, movement.str(), "movement");
  // }
  return T_movement.matrix();
}

Eigen::Matrix4d
filter::LaserNAVCalibration::getScanTransformationFrom(
  pointCloud::Ptr last
  , pointCloud::Ptr latest
  , double* icp_fitness_score
  , double iterations)
{
  Eigen::Matrix4d transformationBetweenScans;

  pcl::copyPointCloud(*last, *last_to_draw);

  if (downsampling) {
    filter::downsample(last);
    filter::downsample(latest);
  }

  if (bilateral_filtering) {
    filter::bilateralFiltering(last);
    filter::bilateralFiltering(latest);
  }

  if (calculate_normals || covariance_sampling || initial_aligning) {
    computeSurfaceNormals(last);
    computeSurfaceNormals(latest);

    if (covariance_sampling) {
      covarianceDownsampling(last);
      covarianceDownsampling(latest);
    }

    if (initial_aligning) {
      pcl::PointCloud< pcl::FPFHSignature33 >::Ptr last_descriptors
        = filter::fastPointFeatureHistogram(last);
      pcl::PointCloud< pcl::FPFHSignature33 >::Ptr latest_descriptors
        = filter::fastPointFeatureHistogram(latest);

      transformationBetweenScans = filter::initialAlignment(last, last_descriptors
                                   , latest, latest_descriptors);
    }
  }

  pcl::copyPointCloud(*last, *last_to_draw);

  switch (ICPMethod) {
  case E_STANDARD:
    transformationBetweenScans = filter::icp(last, latest, icp_fitness_score, iterations);
    break;
  case E_LEVENBERG_MARQUARDT:
    transformationBetweenScans = filter::icpLM(last, latest, icp_fitness_score, iterations);
    break;
  }
  return transformationBetweenScans;
}

double filter::LaserNAVCalibration::evaluateICPFitnessScore(
  pointCloud::Ptr first
  , pointCloud::Ptr second
  , double pos_x
  , double pos_y
  , double pos_z
  , double phi
  , double theta
  , double psi
)
{
  const unsigned int iterate_once = 0;
  double icp_fitness_score = 0.0;

  Eigen::Affine3d T_mounting_pose;
  pcl::getTransformation(pos_x, pos_y, pos_z, phi, theta, psi, T_mounting_pose);

  // create temporary copies from point-clouds
  pointCloud::Ptr temp_first (new pointCloud);
  pcl::copyPointCloud(*first, *temp_first);
  pointCloud::Ptr temp_second (new pointCloud);
  pcl::copyPointCloud(*second, *temp_second);

  // transform both from sensor to carrier with estimated mounting-pose
  pcl::transformPointCloud(*temp_first, *temp_first, T_mounting_pose);
  pcl::transformPointCloud(*temp_second, *temp_second, T_mounting_pose);

  // transform first point-cloud to next pose with given nav-data
  Eigen::Affine3d T_movement = Eigen::Affine3d(getMovementFrom(temp_first, temp_first));
  pcl::transformPointCloud(*temp_first, *temp_first, T_movement);

  // get error-metric depending on the difference between both scans (wrong mounting-pose)
  getScanTransformationFrom(
    temp_first
    , temp_second
    , &icp_fitness_score
    , iterate_once);
  return icp_fitness_score;
}

void filter::LaserNAVCalibration::getMountingPose(
  const double *par
  , int m_dat
  , const void *data
  , double *fvec
  , int *info )
{
  const std::vector<last_and_latest> *curr_data_set
    = reinterpret_cast<const std::vector<last_and_latest>* >(data);
  unsigned int nr_scan_pairs = curr_data_set->size();

  double score_sum = 0.0;

  for (int i = 0; i < nr_scan_pairs; ++i) {

    double icp_fitness_score
      = evaluateICPFitnessScore(
          curr_data_set->at(i).first, curr_data_set->at(i).second
          , par[0], par[1], par[2], par[3], par[4], par[5]);

    fvec[i] = icp_fitness_score;
    if (fvec[i] > worst_score) {
      worst_pair = i;
      worst_score = fvec[i];
    }
    score_sum += fvec[i];

    std::string pose_header = "data_pair;d_x;d_y;d_z;phi;theta;psi;icp_fitness_score";
    std::stringstream pose;
    pose << i
         << par[0] << ";" << par[1] << ";" << par[2] << ";"
         << par[3] << ";" << par[4] << ";" << par[5] << ";"
         << icp_fitness_score;
    plot(pose_header, pose.str(), "estimated_mounting_pose");
  }
  std::cout << ++iterations << "\r" << std::flush;
  avr_cal_err = score_sum / nr_scan_pairs;
}

const pointCloud::Ptr
filter::LaserNAVCalibration::getLatestCloud(void)const
{
  return latest_to_draw;
}

const pointCloud::Ptr
filter::LaserNAVCalibration::getLastCloud(void)const
{
  return last_to_draw;
}

void
filter::LaserNAVCalibration::setDownsampling(bool new_state)
{
  downsampling = new_state;
}

bool
filter::LaserNAVCalibration::getDownsampling(void)
{
  return downsampling;
}

void
filter::LaserNAVCalibration::setBilateralFiltering(bool new_state)
{
  bilateral_filtering = new_state;
}

bool
filter::LaserNAVCalibration::getBilateralFiltering(void)
{
  return bilateral_filtering;
}

void
filter::LaserNAVCalibration::setCalculateNormals(bool new_state)
{
  calculate_normals = new_state;
}

bool
filter::LaserNAVCalibration::getCalculateNormals(void)
{
  return calculate_normals;
}

void
filter::LaserNAVCalibration::setCovarianceSampling(bool new_state)
{
  covariance_sampling = new_state;
}

bool
filter::LaserNAVCalibration::getCovarianceSampling(void)
{
  return covariance_sampling;
}

void
filter::LaserNAVCalibration::setInitialAligning(bool new_state)
{
  initial_aligning = new_state;
}

bool
filter::LaserNAVCalibration::getInitialAligning(void)
{
  return initial_aligning;
}

void
filter::LaserNAVCalibration::addToDataSet(void)
{
  add_to_data_set = true;
}

void
filter::LaserNAVCalibration::resetDataSet(void)
{
  data_set.clear();
}

void
filter::LaserNAVCalibration::optimizeDataSet(void)
{
  std::swap(data_set[worst_pair], data_set.back());
  data_set.pop_back();
}

void
filter::LaserNAVCalibration::setScansBetweenLastAndLatest(
  unsigned int scans_between_last_and_latest)
{
  this->scans_between_last_and_latest = scans_between_last_and_latest;
}

void
filter::LaserNAVCalibration::update(pointCloud::Ptr cloud_to_draw)
{
  iterations = 0;
  if (plot_all_movements) {
    if (0 == scan_counter) {
      if (cloud_to_draw && !cloud_to_draw->points.empty()) {
        pcl::copyPointCloud(*cloud_to_draw, *last);
        // boost::static_pointer_cast<pointCloud>(last)->setNavData(
        //   boost::static_pointer_cast<pointCloud>(cloud_to_draw)->nav_data);
        ++scan_counter;
      }
    }
    else if (scans_between_last_and_latest == scan_counter) {
      if (cloud_to_draw && !cloud_to_draw->points.empty()) {
        pcl::copyPointCloud(*cloud_to_draw, *latest);
        // boost::static_pointer_cast<pointCloud>(latest)->setNavData(
        //   boost::static_pointer_cast<pointCloud>(cloud_to_draw)->nav_data);
        double d_x, d_y, d_z, d_phi, d_theta, d_psi;
        pcl::getTranslationAndEulerAngles(
          Eigen::Affine3d(getMovementFrom(last, latest)), d_x, d_y, d_z, d_phi, d_theta, d_psi);
        if (sampled_scans == 170 || sampled_scans == 200 || sampled_scans == 75
            || sampled_scans == 140 || sampled_scans == 60 || sampled_scans == 190) {
          pointCloud::Ptr temp_last (new pointCloud);
          pointCloud::Ptr temp_latest (new pointCloud);
          pcl::copyPointCloud(*last, *temp_last);
          pcl::copyPointCloud(*latest, *temp_latest);
          data_set.push_back(last_and_latest(temp_last, temp_latest));
//          double icp_fitness_score;
//          getScanTransformationFrom(
//            temp_last
//            , temp_latest
//            , &icp_fitness_score
//            , 1);
          ++nr_scan_pairs;
        }
        ++sampled_scans;
        scan_counter = 0;
      }
    }
    ++scan_counter;
  }

//  if(add_to_data_set) {
//    switch (scan_counter) {
//    case 0: {
//      if(cloud_to_draw && !cloud_to_draw->points.empty()) {
//        pcl::copyPointCloud(*cloud_to_draw, *last);
//        boost::static_pointer_cast<pointCloud>(last)->setNavData(
//          boost::static_pointer_cast<pointCloud>(cloud_to_draw)->nav_data);
//        ++scan_counter;
//      }
//      break;
//    }
//    case scans_between_last_and_latest: {
//      if(cloud_to_draw && !cloud_to_draw->points.empty()) {
//        pcl::copyPointCloud(*cloud_to_draw, *latest);
//        boost::static_pointer_cast<pointCloud>(latest)->setNavData(
//          boost::static_pointer_cast<pointCloud>(cloud_to_draw)->nav_data);
//        pointCloud::Ptr temp_last (new pointCloud);
//        pointCloud::Ptr temp_latest (new pointCloud);
//        pcl::copyPointCloud(*last, *temp_last);
//        pcl::copyPointCloud(*latest, *temp_latest);
//        data_set.push_back(last_and_latest(temp_last, temp_latest));
//        ++nr_scan_pairs;
//        add_to_data_set = false;
//        scan_counter = 0;
//      }
//      break;
//    }
//    default:
//      ++scan_counter;
//    }

  if (nr_scan_pairs >= nr_unknowns && calibration_wanted) {
    // set the mounting_pose initially
    if (init) {
      mounting_pose[0] = pos_x;
      mounting_pose[1] = pos_y;
      mounting_pose[2] = pos_z;
      mounting_pose[3] = phi;
      mounting_pose[4] = theta;
      mounting_pose[5] = psi;
      init = false;
    }

    /* auxiliary parameters */
    lm_status_struct status;
    lm_control_struct control = lm_control_double;
    control.verbosity = 3;

//      control.ftol = LM_USERTOL;
//      control.xtol = LM_USERTOL;
//      control.gtol = LM_USERTOL;
    control.epsilon = math::c_PI;

    /* perform the fit */
    printf( "Fitting:\n" );
    lmmin( nr_unknowns, mounting_pose, nr_scan_pairs, (const void*)&data_set
           , LaserNAVCalibration::getMountingPose
           , &control, &status );

    std::cout << "Iterations: " << status.nfev << std::endl;

    print(
      mounting_pose[0], mounting_pose[1], mounting_pose[2]
      , mounting_pose[3], mounting_pose[4], mounting_pose[5]
      , "Pose");

    plot_all_movements = false;

    // maybe we are trapped in a local-minimum so re-init the solution
    if (avr_cal_err > 10.0) {
      optimizeDataSet();

      Eigen::Affine3d T_new_init;
      Eigen::Affine3d T_scan_pairs;
      double smallest_score = 100.0;
      double scan_pairs_score = 100.0;
      for (unsigned int i = 0; i < nr_scan_pairs; ++i) {
        T_scan_pairs = getScanTransformationFrom(
                         data_set.at(i).first
                         , data_set.at(i).second
                         , &scan_pairs_score
                         , 20);
        if (scan_pairs_score < smallest_score) {
          smallest_score = scan_pairs_score;
          T_new_init = T_scan_pairs;
        }
      }
      pcl::getTranslationAndEulerAngles(
        T_new_init
        , mounting_pose[0], mounting_pose[1], mounting_pose[2]
        , mounting_pose[3], mounting_pose[4], mounting_pose[5]);

      print(&T_new_init, "new-init");
    } else {
      calibration_wanted = false;
    }
  }
// ScannerOverviewFilter::update();
}