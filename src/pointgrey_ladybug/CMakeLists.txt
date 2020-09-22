cmake_minimum_required(VERSION 2.8.12)

project(pointgrey_ladybug)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	tf
	cv_bridge
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -g ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
link_directories(/usr/lib/ladybug)
catkin_package()

###########
## Build ##
###########

include_directories(
			/usr/include/ladybug
			src/ladybug/
			include
			${catkin_INCLUDE_DIRS}
			${OpenCV_INCLUDE_DIRS}
)
if(EXISTS "/usr/include/ladybug")

	add_executable(ladybug_camera_jpeg_file
			src/ladybug/ladybug_driver_jpeg.cpp
			include/ConfigurationLoader.cpp
			include/ConfigurationLoader.cpp
			include/ImageGrabber.cpp
			include/LadybugFunctors.cpp
			include/ImageRecorder.cpp
			include/LadybugRecorderConsoleConfiguration.cpp
			include/stdafx.cpp
	)
	target_link_libraries(ladybug_camera_jpeg_file
			${catkin_LIBRARIES}
			${OpenCV_LIBS}
			flycapture
			ladybug
			/usr/lib/x86_64-linux-gnu/libxerces-c-3.1.so
	)

	add_executable(ladybug_camera_raw_display
			src/ladybug/ladybug_driver_raw.cpp
			)
	target_link_libraries(ladybug_camera_raw_display
			${catkin_LIBRARIES}
			${OpenCV_LIBS}
			flycapture
			ladybug
			)

	add_executable(pgr2tiff
			src/ladybug/pgr2tiff.cpp
			)
	target_link_libraries(pgr2tiff
			flycapture
			ladybug
			)

else()
	message("'SDK for Ladybug' is not installed. 'ladybug_camera' will not be built.")
endif()
