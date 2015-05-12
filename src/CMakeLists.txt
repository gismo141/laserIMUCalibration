# find_package(PCL 1.2 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# add_executable(icp_with_noise icp_with_noise.cpp)
# target_link_libraries(icp_with_noise ${PCL_LIBRARIES})

include_directories("object")
include_directories("filter")
include_directories("gui")

add_library(laserIMUCalibrationLib
	./filter/calibrator.cpp
	./filter/noiseTool.cpp
	./gui/visualizer.cpp)

qt5_use_modules(laserIMUCalibrationLib Widgets)