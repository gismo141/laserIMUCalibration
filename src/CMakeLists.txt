include_directories("object")
include_directories("filter")
include_directories("gui")

add_library(laserIMUCalibrationLib
	./filter/calibrator.cpp
	./filter/noiseTool.cpp
	./gui/visualizer.cpp)

qt5_use_modules(laserIMUCalibrationLib Widgets)