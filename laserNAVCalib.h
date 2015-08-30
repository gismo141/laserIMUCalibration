/**
 * @file    laserNAVCalib.h
 * @brief   This file introduces the class LaserNAVCalibration which is
 *          used to calibrate the laser-pose with nav-data.
 *
 * Copyright 2015 <michael.r141@gmail.com>
 *
 * @author  [Michael Riedel](mailto:michael.r141@gmail.com?subject=laserNAVCalib.h)
 *
 **/

#ifndef _LASER_NAV_CALIB_H
#define _LASER_NAV_CALIB_H

#include <vector>

// convert rad 2 deg
#include "mathConstants.h"

#include <pcl/common/common_headers.h>

/**
 * @brief Typedef for shorter call of point-cloud-pointer-type
 */
typedef pcl::PointCloud<pcl::PointNormal> pointCloud;

/**
 * @brief Typedef for shorter call (used as data-set for the levenberg-marquardt data)
 */
typedef std::pair<pointCloud::Ptr, pointCloud::Ptr> last_and_latest;

namespace filter
{
/**
 * @brief   Filter to calibrate the Laser mounting Pose according to NAV-data.
 * @details
 * @author  Michael Riedel
 * @date    2015-06-29
 *
 */
class LaserNAVCalibration
{
public:
  /**
   * @brief Selector for the calibration-method (not supported atm)
   */
  enum e_registrationMethod {
    E_STANDARD
    , E_LEVENBERG_MARQUARDT
  };
private:
  /**
   * @brief Number of unknowns @see update()
   */
  static const unsigned int nr_unknowns = 6;
  /**
   * @brief Number of scans between point-cloud-pairs (those are simply dropped)
   */
  static unsigned int scans_between_last_and_latest;
  /**
   * @brief Contains the number of all sampled scans
   */
  double sampled_scans;
  /**
   * @brief Counts the scans between last and latest point-cloud
   */
  unsigned int scan_counter;
  /**
   * @brief Check all scans and plot all movements to a file
   * @details This variable is set to false, when optimization has completed (no multi-iteration)
   */
  bool plot_all_movements;
  /**
   * @brief Set to true when calibration is wanted
   */
  bool calibration_wanted;

  /**
   * @brief The data-array used by the Levenberg-Marquardt lmmin-optimizer
   */
  double mounting_pose[nr_unknowns];
  /**
   * @brief Class-wide latest position (this is **altered after** optimization)
   */
  double pos_x, pos_y, pos_z, phi, theta, psi;

  /**
   * @brief The cloud-ptr provided by another source (work is based on this)
   */
  pointCloud::Ptr cloud_to_draw;
  /**
   * @brief Cloud-ptr that stores the last point-cloud
   */
  pointCloud::Ptr last;
  /**
   * @brief Cloud-ptr that stores the latest point-cloud
   */
  pointCloud::Ptr latest;
  /**
   * @brief Cloud-ptr that stores the last point-cloud that will be drawn (stable, switched after registration-process)
   */
  static pointCloud::Ptr last_to_draw;
  /**
   * @brief Cloud-ptr that stores the latest point-cloud that will be drawn (stable, switched after registration-process)
   */
  static pointCloud::Ptr latest_to_draw;
  /**
   * @brief Holds the data-set of point-cloud-pairs (known as key-sequences in the thesis)
   */
  std::vector< last_and_latest > data_set;
  /**
   * @brief holds the number of elements in the data-set (std::vector.size() takes too long)
   * @details not needed anymore in C++11
   */
  unsigned int nr_scan_pairs;
  /**
   * @brief Index of the point-cloud-pair with the worst ICP-fitness-score
   * @details The pair at this index will be purged from the data-set when the average-error was too bad in the optimization
   */
  static size_t worst_pair;
  /**
   * @brief Holds the worst ICP-fitness-score for the according point-cloud-pair
   */
  static double worst_score;
  /**
   * @brief Mean of the summed ICP-fitness-score's (updated each iteration)
   */
  static double avr_cal_err;
  /**
   * @brief Selector for the calibration-method (not supported atm)
   */
  static e_registrationMethod ICPMethod;
  /**
   * @brief When true: all point-clouds are downsampled using Voxel-Grids
   */
  static bool downsampling;
  /**
   * @brief When true: bilateral-filtering is used (computes fpfh-features!)
   */
  static bool bilateral_filtering;
  /**
   * @brief When true: calculate normals (basis for covariance-sampling and initial-alignment)
   */
  static bool calculate_normals;
  /**
   * @brief When true: Samples the point-cloud using covariance-matrices
   */
  static bool covariance_sampling;
  /**
   * @brief When true: Initial Alignment is calculated using Sample-Consensus (**untested**)
   */
  static bool initial_aligning;
  /**
   * @brief Set to true on button press when the user requested to add a data-set to the key-sequences
   * @details Reset right after the data-set was added
   */
  bool add_to_data_set;
  /**
   * @brief Initializes the optimization-array for the Levenberg-Marquardt using ctor. given parameters
   */
  bool init;
  /**
   * @brief Variable to store a class-wide filename (can be used to group different plots according to a naming scheme)
   */
  std::stringstream filename;

  /**
   * @brief Print the given transformation on command-line-interface
   * @param pos_x The x-coordinate
   * @param pos_y The y-coordinate
   * @param pos_z The z-coordinate
   * @param phi The Euler-angel around x-axis (input in rad, output in degree)
   * @param theta The Euler-angle around y-axis (input in rad, output in degree)
   * @param psi The Euler-angle around z-axis (input in rad, output in degree)
   * @param name Name that is prepended to the output (great to differentiate multiple prints)
   */
  void print(
    double pos_x, double pos_y, double pos_z
    , double phi, double theta, double psi
    , std::string name = "Transformation");

  /**
   * @brief Print the given transformation on command-line-interface
   * @details Calls print(Eigen::Matrix4d,...)
   *
   * @param transformation_to_print The transformation to print
   * @param name Name that is prepended to the output (great to differentiate multiple prints)
   */
  void print(
    Eigen::Affine3d* transformation_to_print
    , std::string name = "Transformation");

  /**
   * @brief Print the given transformation on command-line-interface
   * @details Calls print(pos_x, pos_y,...)
   *
   * @param transformation_to_print The transformation to print
   * @param name Name that is prepended to the output (great to differentiate multiple prints)
   */
  void print(
    Eigen::Matrix4d* transformation_to_print
    , std::string name = "Transformation");

  /**
   * @brief Checks if the given filename-at-path exists
   *
   * @param name The path + filename
   * @returns
   *            | value | meaning             |
   *            |:------|:--------------------|
   *            | true  | file exists         |
   *            | false | file does not exist |
   */
  static bool exists(const std::string& name);

  /**
   * @brief Plot the given header + data to file
   * @details Checks if the given filename exists:
   *
   *          - **true** the data will be appended,
   *          - **false** a new file is created, the header + data will be added and file is closed
   *
   * @param header The formatted header (use the **same separators** as in data!)
   * @param data The formatted data (use the **same separators** as in header)
   * @param filename The name and the path where the file should be stored
   */
  static void plot(
    std::string header
    , std::string data
    , std::string filename);

  /**
   * @brief Retrieves the transformation from last to latest point-cloud
   * @details Calls the filtering functions inside if the according boolean is set.
   *          ATTENTION! When filtering functions are activated, the clouds will be altered!
   *
   * @param last PointCloud::Ptr to the first point-cloud
   * @param latest PointCloud::Ptr to the second point-cloud
   * @param icp_fitness_score The variable where the resulting score should be stored
   * @param iterations Number of iterations the ICP should pass in order to refine the transformation
   * @return The scan transformation from last to latest
   */
  static Eigen::Matrix4d getScanTransformationFrom(
    pointCloud::Ptr last
    , pointCloud::Ptr latest
    , double* icp_fitness_score
    , double iterations);

  /**
   * @brief Retrieves the movement of the Navigation-Data between the last and latest point-cloud
   * @details No data will be altered internally (just subtraction of coordinates and angles)
   *
   * @param last PointCloud::Ptr to the first point-cloud
   * @param latest PointCloud::Ptr to the second point-cloud
   *
   * @return The movement transformation from last to latest
   */
  static Eigen::Matrix4d getMovementFrom(
    pointCloud::Ptr last
    , pointCloud::Ptr latest);

  static double evaluateICPFitnessScore(
    pointCloud::Ptr first
    , pointCloud::Ptr second
    , double pos_x
    , double pos_y
    , double pos_z
    , double phi
    , double theta
    , double psi
  );

  /**
   * @brief Optimized the mounting-pose as part of the Levenberg-Marquardt
   * @details DO NOT CALL THIS METHOD YOURSELF! Create an instance of math::lmmin as shown in update()
   *
   * @param par pos_x, pos_y, pos_z, phi, theta, psi that will be optimised
   * @param m_dat NOT USED BY THIS FUNCTION
   * @param data the vector of selected point-cloud-pairs
   * @param fvec Error-vector (used to store the new ICP-fitness-scores)
   * @param info NOT USED BY THIS FUNCTION
   */
  static void getMountingPose(
    const double *par
    , int m_dat
    , const void *data
    , double *fvec
    , int *info );

public:
  /**
   * @brief std. ctor.
   * @details Creates an instance for calibrating mountingpose for the lidar according to navigation-data.
   *          Manually measured mounting-pose can be passed as optional arguments in distances and degrees.
   *
   * @param pos_x x-coordinate in inital position
   * @param pos_y y-coordinate in initial position
   * @param pos_z z-coordinate in initial position
   * @param phi phi in initial pose
   * @param theta theta in initial pose
   * @param psi psi in initial pose
   */
  LaserNAVCalibration(
    double pos_x = 0.0, double pos_y = 0.0, double pos_z = 0.0
        , double phi = math::c_DEG_TO_RAD * 0.0
                       , double theta = math::c_DEG_TO_RAD * 0.0
                                        , double psi = math::c_DEG_TO_RAD * 0.0);

  /**
   * @brief Virtual std. dtor.
   */
  virtual ~LaserNAVCalibration();

  /**
   * @brief Returns the latest stable cloud
   * @details This is the extra copy called latest_to_draw
   * @return Returns latest_to_draw
   */
  const pointCloud::Ptr getLatestCloud(void)const;

  /**
   * @brief Returns the last stable cloud
   * @details This is the extra copy called last_to_draw
   * @return Returns last_to_draw
   */
  const pointCloud::Ptr getLastCloud(void)const;

  /**
   * @brief Sets the filter voxel-grid
   *
   * @param new_state The new state that will be set
   */
  void setDownsampling(bool new_state);
  /**
   * @brief Retrieves if the voxel-grid filter is activated
   * @return The state of the voxel-grid filter
   */
  bool getDownsampling(void);

  /**
   * @brief Sets the bilateral filter
   * @details NOT USED AT THE MOMENT
   *
   * @param new_state The new state that will be set
   */
  void setBilateralFiltering(bool new_state);
  /**
   * @brief Retrieves if the bilateral filter is activated
   * @details NOT USED AT THE MOMENT
   * @return The state of the bilateral filter
   */
  bool getBilateralFiltering(void);

  /**
   * @brief Sets the calculation of point-normals
   *
   * @param new_state The new state that will be set
   */
  void setCalculateNormals(bool new_state);
  /**
   * @brief Retrieves if the bilateral filter is activated
   * @details NOT USED AT THE MOMENT
   * @return The state of the bilateral filter
   */
  bool getCalculateNormals(void);

  /**
   * @brief Sets the covariance-sampling of point-normals
   *
   * @param new_state The new state that will be set
   */
  void setCovarianceSampling(bool new_state);
  /**
   * @brief Retrieves if the covariance-sampling is activated
   * @return The state of the covariance-sampling filter
   */
  bool getCovarianceSampling(void);

  /**
   * @brief Sets the initial alignment of point-normals
   *
   * @param new_state The new state that will be set
   */
  void setInitialAligning(bool new_state);
  /**
   * @brief Retrieves if the initial alignment is activated
   * @details NOT USED AT THE MOMENT
   * @return The state of the bilateral filter
   */
  bool getInitialAligning(void);

  /**
   * @brief Sets the boolean to add the point-clouds at the active index to the data-set-pairs
   */
  void addToDataSet(void);

  /**
   * @brief Reset/Remove all point-cloud-pairs from the data-set
   */
  void resetDataSet(void);

  /**
   * @brief Deletes the worst-pair according to its ICP-fitness-score
   */
  void optimizeDataSet(void);

  /**
   * @brief Sets the number of scans to skip between one registration.
   * @details The number should match the speed of the sensor-carrier.
   * 
   * 
   * @param scans_between_last_and_latest The number of scans to skip.
   */
  void setScansBetweenLastAndLatest(unsigned int scans_between_last_and_latest);

  /**
   * @brief Updates the mounting-pose using Levenberg-Marquardt non-linear optimization
   * @param cloud_to_draw The cloud-ptr provided by another source (work is based on this)
   */
  virtual void update(pointCloud::Ptr cloud_to_draw);
};
}  // filter
#endif  // _LASER_NAV_CALIB_H
