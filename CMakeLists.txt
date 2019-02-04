cmake_minimum_required(VERSION 2.8.3)
project(moveit_collision_inspector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	ur10_moveit_config
	roslib
	actionlib
    moveit_ros_move_group
    moveit_ros_visualization
    moveit_msgs
)

find_package(Boost COMPONENTS filesystem system algorithm)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveit_collision_inspector
#  CATKIN_DEPENDS ur10_moveit_config
#  DEPENDS system_lib
)

include_directories(
	# include
	${catkin_INCLUDE_DIRS}
)

add_executable(sender src/sender.cpp)
target_link_libraries(sender
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)