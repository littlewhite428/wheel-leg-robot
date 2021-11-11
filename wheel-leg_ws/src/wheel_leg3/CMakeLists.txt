cmake_minimum_required(VERSION 2.8.3)

project(wheel_leg3)

# find_package(catkin REQUIRED)

# catkin_package()

find_package(catkin REQUIRED
	rospy
	roslaunch
	# rospy
	roscpp
	std_msgs
	sensor_msgs
	nav_msgs
	message_generation
)
add_message_files(
  FILES
  Balance.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
	CATKIN_DEPENDS message_runtime
)
foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
