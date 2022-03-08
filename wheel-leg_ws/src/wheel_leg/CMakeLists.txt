cmake_minimum_required(VERSION 2.8.3)

project(wheel_leg)

# find_package(catkin REQUIRED)

# catkin_package()

find_package(catkin REQUIRED
	rospy
	roslaunch
	# rospy
	roscpp
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	message_generation
)
add_message_files(
  FILES
  Balance.msg
  Footprint.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
	include	
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
)
## Declare a cpp executable
add_executable(lqr_control_node 
	src/lqr_control_node.cpp
)
target_link_libraries(lqr_control_node ${catkin_LIBRARIES})

add_executable(biped_control_node 
	src/kinematics.cpp
	src/biped_control_node.cpp
)
target_link_libraries(biped_control_node ${catkin_LIBRARIES})
foreach(dir config launch meshes meshes2 urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
