cmake_minimum_required(VERSION 2.8.3)
project(catchbot)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	trajectory_msgs)

catkin_package()

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(catchbot_node src/catchbot_node.cpp src/kinematics.cpp)

# add_executable(catchbot_node src/catchbot_node.cpp)

add_dependencies(catchbot_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(catchbot_node ${catkin_LIBRARIES})