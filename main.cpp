/**
 * Copyright 2015 <michael.r141@gmail.com>
 *
 * @file	main.cpp
 * @author 	[Michael Riedel](mailto:michael.r141@gmail.com?subject=main.cpp)
 */

#include <dirent.h>
#include <vector>
#include <string>
#include <iostream>

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include "laserNAVCalib.h"

//convenient typedefs
typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<PointT> pointCloud;

std::vector<std::string> open(std::string path = ".") {
	DIR* dir;
	dirent* pdir;
	std::vector<std::string> scans;

	dir = opendir(path.c_str());

	while ((pdir = readdir(dir))) {
		scans.push_back(path + pdir->d_name);
	}

	scans.erase(std::find(scans.begin(), scans.end(), path + "."));
	scans.erase(std::find(scans.begin(), scans.end(), path + ".."));

	return scans;
}

int main(int argc, char* argv[]) {
	if (argc < 2) {
		return EXIT_FAILURE;
	}

	std::vector<std::string> scans = open(argv[1]);
	filter::LaserNAVCalibration calibrator(0.43, 0, -0.15, 90, 0, 90);

	for (auto scan : scans) {
		switch (argc) {
		case 2:
			calibrator.setScansBetweenLastAndLatest(atoi(argv[2]));
		}
		pointCloud::Ptr new_cloud_data (new pointCloud);
		pcl::io::loadPCDFile(scan, *new_cloud_data);
		calibrator.update(new_cloud_data);
	}
	return 1;
}
