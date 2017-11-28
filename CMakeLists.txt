cmake_minimum_required(VERSION 2.8.3)
project(multi-velodyne)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  rosbag
  roscpp
  rospy
  std_msgs
  velodyne_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES multi-velodyne
  CATKIN_DEPENDS 
		pcl_ros 
		rosbag
		roscpp
		rospy
		std_msgs
		velodyne_msgs
  DEPENDS 
		system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
		${catkin_INCLUDE_DIRS}
		# ${Boost_INCLUDE_DIRS}
)

