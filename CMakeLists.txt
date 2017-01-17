cmake_minimum_required(VERSION 2.8)
project(vertical_stereo)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	image_transport
	cv_bridge
	camera_calibration_parsers
	rosbag
)
find_package(Boost REQUIRED COMPONENTS
	thread
	filesystem
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(EIGEN3_INCLUDE_DIR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -std=c++11")

catkin_package()

add_executable(vertical_stereo
                src/vertical_stereo.cpp
)
target_link_libraries(vertical_stereo
        ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

add_executable(rotate_imgs
                src/rotate_imgs.cpp
)
target_link_libraries(rotate_imgs
        ${catkin_LIBRARIES} ${OpenCV_LIBS}
)
