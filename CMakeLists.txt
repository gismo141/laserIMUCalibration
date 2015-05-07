cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project(laserIMUCalibration)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(VERSION 0.1-alpha)

add_definitions(-std=c++11)

add_subdirectory(Documentation) # doxygen documentation
add_subdirectory(src)

if( APPLE )
	set(PROGNAME laserIMUCalibration)
	# set(MACOSX_BUNDLE_ICON_FILE ${PROGNAME}.icns )
	set(MACOSX_BUNDLE_SHORT_VERSION_STRING VERSION )
    set(MACOSX_BUNDLE_VERSION VERSION )
    set(MACOSX_BUNDLE_LONG_VERSION_STRING Version VERSION )
else( APPLE )
	set(PROGNAME laserIMUCalibration)
endif( APPLE )

find_package(Qt5Core REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("inc")

add_subdirectory(Documentation) # doxygen documentation
add_subdirectory(src)			# create library

set(_MOC_HDRS					# moc all headers
	inc/*.h)

qt5_wrap_cpp(_MOC_SRCS ${_MOC_HDRS})

if(APPLE)
	add_executable(
		${PROGNAME} MACOSX_BUNDLE main.cpp ${_SRCS} ${_MOC_SRCS}
	)
    add_custom_command(TARGET ${PROGNAME} POST_BUILD
    	command mkdir ARGS -p
    	${CMAKE_CURRENT_BINARY_DIR}/${PROGNAME}.app/Contents/Resources
    	command cp ARGS
    	../${MACOSX_BUNDLE_ICON_FILE}
    	${CMAKE_CURRENT_BINARY_DIR}/${PROGNAME}.app/Contents/Resources
    	command cp ARGS
    	../*.md
    	${CMAKE_CURRENT_BINARY_DIR}/${PROGNAME}.app/Contents/Resources
    )
elseif(WIN32)
	add_executable(
		${PROGNAME} WIN32 main.cpp ${_SRCS} ${_MOC_SRCS}
	)
else(APPLE)
	add_executable(
		${PROGNAME} main.cpp ${_SRCS} ${_MOC_SRCS}
	)
endif(APPLE)

target_link_libraries(${PROGNAME} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${PROGNAME}Lib)
qt5_use_modules(${PROGNAME} Widgets)
