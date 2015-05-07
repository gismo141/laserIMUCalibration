find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(icp_with_noise icp_with_noise.cpp)
target_link_libraries(icp_with_noise ${PCL_LIBRARIES})

add_executable(interactive_icp interactive_icp.cpp)
target_link_libraries(interactive_icp ${PCL_LIBRARIES})

add_executable(pcd_write pcd_write.cpp)
target_link_libraries(pcd_write ${PCL_LIBRARIES})

add_executable(pcl_icp pcl_icp.cpp)
target_link_libraries(pcl_icp ${PCL_LIBRARIES})

add_executable(rotationToEuler rotationToEuler.cpp)
target_link_libraries(rotationToEuler ${PCL_LIBRARIES})

add_executable(scan_pcl_visualizer scan_pcl_visualizer.cpp)
target_link_libraries(scan_pcl_visualizer ${PCL_LIBRARIES})

add_executable(statistical_noise_removal statistical_noise_removal.cpp)
target_link_libraries(statistical_noise_removal ${PCL_LIBRARIES})