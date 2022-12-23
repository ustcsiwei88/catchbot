cmake_minimum_required(VERSION 2.8.3)
project(catchbot)

find_package(ur_rtde REQUIRED)

include_directories(include)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)


link_directories(lib)
add_executable(catchbot_node src/catchbot_node.cpp src/kinematics.cpp)
target_link_libraries(catchbot_node ${catkin_LIBRARIES} ur_rtde::rtde NatNet)
