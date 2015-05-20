# Automated LiDAR-IMU Calibration with ICP

This program is part of my [mastersthesis](https://github.com/gismo141/mastersthesis).

The objective is to provide an automated calibration method that uses movement data collected by Inertial Measurement Units (IMU)-sensors to calibrate the mounting-pose of a Light Detection and Ranging (LiDAR)-scanner.

## Background

Autonomous systems are meant to solve tasks that are dangerous or boring for humans. Algorithms where implemented to mimic human-actions. The basis for those actions is the ability to sense the environment. Optical sensors like cameras or LiDAR are used to generate different features of the environment. IMU composed of Gyroscopes and Global Positioning Systems (GPS) are used to determine the pose and position in multidimensional spaces.

To generate a close representation of the real world, multiple sensors are used at the same time. The combination of their data is called Sensor-Fusion. Sensor-Fusion enables the system to evaluate actions according to the actual situation, even if some sensors deliver faulty inputs which are compensated by others. The basis for effective Sensor-Fusion is concrete knowledge about the sensor and its pose and position on the autonomous system. Only then it is possible to transform the collected data into a common coordinatesystem.

Due to the different constructions, missions and load-configurations of Autonomous Systems it is nearly impossible and tedious to calibrate the system by hand.

## Approach

The Incremental Closest Point (ICP)-Algorithm is used to detect the carriers movement between successive LiDAR-scans. The ICP returns a transformation between both scans and an error or fitness-score. The transformation is then compared to the collected data by the IMU. Because the LiDAR and the IMU are mounted fixed on the same carrier - the difference between both, the ICP-calculated and the IMU-measured movement, is supposed to be the miscalibration in the LiDARs pose and position.

The calculated difference is then added to the old pose and position information and the process continues. We suppose, that with multiple iterations it should be possible to determine the nearly correct pose and position.

## Implementation

This repository contains the developed reaserch code. It is meant to be used standalone to evaluate different approaches. The implementation uses Qt5 for a Graphical User Interface (GUI), VTK for algorithm visualisation and the Point-Cloud-Library (PCL) for pointcloud management and different algorithms.

# Installation

## Dependencies

**PLEASE READ THE COMPLETE SECTION BEFORE THE INSTALLATION**

For Max OS X it is proposed to use [Homebrew](http://brew.sh) for the installation of the the software and its dependencies.

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

The command-line-syntax for the installation with Homebrew is:

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

The `*.app`-Bundle is used as the GUI-Binary while the other should be used with the commandline. This behaviour will be changed after implementing a sufficient GUI.
