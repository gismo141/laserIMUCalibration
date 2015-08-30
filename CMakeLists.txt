cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project(laserNAVCalibration)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(PCL 1.7.2 REQUIRED COMPONENTS common io registration features surface filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "")
add_definitions(-Wall)

set(PCL_ALL_REQUIRED_LIBRARIES
	${PCL_COMMON_LIBRARIES}
	${PCL_IO_LIBRARIES}
	${PCL_REGISTRATION_LIBRARIES}
	${PCL_FEATURES_LIBRARIES}
	${PCL_FILTERS_LIBRARIES}
	${PCL_SURFACE_LIBRARIES})

list(REMOVE_ITEM PCL_ALL_REQUIRED_LIBRARIES "debug")
list(REMOVE_ITEM PCL_ALL_REQUIRED_LIBRARIES "optimized")

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME ON)

find_package(Boost 1.49 COMPONENTS system filesystem REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

# doxygen documentation
add_subdirectory(Documentation)

add_executable(laserNAVCalibration main.cpp mathConstants.cpp pointcloudRegistration.cpp lmmin.c laserNAVCalib.cpp)
target_link_libraries(laserNAVCalibration ${CMAKE_CXX_FLAGS} ${PCL_ALL_REQUIRED_LIBRARIES} ${Boost_LIBRARIES})
