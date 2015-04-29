include_directories("model")
include_directories("control")
include_directories("view")

add_library(laserIMUCalibrationLib
	./control/goICP.cpp
	./control/icp_with_noise.cpp
	./control/pcd_write.cpp)

qt5_use_modules(laserIMUCalibrationLib Widgets)