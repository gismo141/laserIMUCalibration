# Automated LiDAR-IMU Calibration with ICP

I wrote his program as part of my [mastersthesis](https://github.com/gismo141/mastersthesis).

This provides an automated calibration method that uses movement data collected by Inertial Measurement Units (IMU)-sensors to calibrate the mounting-pose of a Light Detection and Ranging (LiDAR)-scanner.

## Background

Autonomous systems solve dangerous or boring tasks via different algorithms to mimic human-actions. Optical sensors like cameras or LiDAR enable the system to sense the environment and to extract different features of the environment. IMU-Systems determine its pose and position in multidimensional spaces with gyroscopes and accelerometers.

Multiple sensors, used as a sensor-system, generate a close representation of the real world, called Sensor-Fusion. Sensor-Fusion enables the system to evaluate actions according to the actual situation and compensate single faulty sensors. Effective Sensor-Fusion needs concrete knowledge about the sensors and their pose and position on the carrier, the autonomous system. The autonomous system then combines the calibrated sensor-data in a common coordinate-system.

The plethora of different constructions, missions and load-configurations of Autonomous Systems makes it difficult to calibrate the sensors by human-hand.

## Approach

The Incremental Closest Point (ICP)-Algorithm registers a model point-cloud with a target point-cloud and returns the transformation and an error- or fitness-score. We us de ICP-Algorithm to detect the carriers movement between successive LiDAR-scans. We then compare the transformation calculated by the ICP with the collected movement data by the IMU. If a difference between both occurs, we use that information as new calibration-input for the LiDARs pose and position.

We suppose, that with multiple iterations it should be possible to determine the nearly correct pose and position between the LiDAR and the IMU.  

## Implementation

This repository contains the developed research code. The implementation uses Qt5 for a Graphical User Interface (GUI), VTK for algorithm visualisation and the Point-Cloud-Library (PCL) for pointcloud management and different algorithms.

# Installation

## Dependencies

**PLEASE READ THE COMPLETE SECTION BEFORE THE INSTALLATION**

To install the software and its dependencies on Mac OS X, use the package manger [Homebrew](http://brew.sh).

After the installation of Homebrew you should install the software according to the table [#tab:needed-software].

Table: Needed software with its options {#tab:needed-software}

| Package  | Version | Options for Homebrew |
| :---     | :-----  | :--                  |
| CMake    | 3.2.2   |                      |
| Qt5      | 5.4.1   |                      |
| VTK      | 6.2.0   | `--with-qt5 --c++11` |
| PCL      | 1.7.2   |                      |
| GraphViz | 2.38.0  |                      |
| Doxygen  | 1.8.9.1 |                      |

Homebrews command-line-syntax:

```Shell
brew install <Package> [Options]
```

e.g. for VTK:

```Shell
brew install vtk --with-qt5 --c++11
```

## Compilation

```Shell
git clone git@github.com:gismo141/laserIMUCalibration
cd laserIMUCalibration
mkdir build && cd build
cmake ..
make
```

When everything went as it should, you will have two binaries in the build-folder:

- `laserIMUCalibration.app`
- `laserIMUCalibration_noBundle`

# Usage

## Fly through mutliple LiDAR-scans

```Shell
./laserIMUCalibration_noBundle <PATH_TO_FOLDER_WITH_PCD_FILES>
```

This opens the interactive VTK-visualizer with consecutive presentation of each LiDAR-scan. The use can move, rotate and scale the scene with the mouse.

### Planned Improvements

- Implement a slider to show specific scans
- Activate different filters on-the-fly

## Plot the ICP-fitnessscore for different intervals

```Shell
./laserIMUCalibration_noBundle <PATH_TO_FOLDER_WITH_PCD_FILES> <INTERVAL_BETWEEN_SCANS>
```

This launches the ICP-algorithm and calculates the transformation between every LiDAR-scan and its `<INTERVAL_BETWEEN_SCANS>` successor. A small percentage-indicator represents the amount of processed scans divided by the total to provide a little feedback on the calculation-intensive task. While processing, the program writes its results to the semicolon-separated-file `icp_fitness_<INTERVAL_BETWEEN_SCANS>.csv` in the executed folder. The first row is the unix-timestamp of the first scan, the second row is the unix-timestamp of the second scan and the third is the ICP-fitnessscore. The fitnessscore is according to the [PCL](http://docs.pointclouds.org/trunk/classpcl_1_1_registration.html#ab26742c383b6f5e86fb96a236fb08728) the sum of the squared distances divided by its total.

An example `icp_fitness_1.csv` may look like:

```csv
1412242263.910747;1412242264.910452;0.135022
1412242264.010833;1412242265.010031;0.288052
1412242264.110734;1412242265.109657;0.107826
1412242264.210728;1412242265.209419;0.256894
```
