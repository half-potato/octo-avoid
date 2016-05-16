cmake_minimum_required(VERSION 2.8.11)
project(octo_avoid)
#set(CMAKE_CXX_COMPILER "g++")
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS -std=c++11)

find_package(catkin REQUIRED COMPONENTS
	rospy
	copter_control
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES octo_avoid
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

#########
## ROS ##
#########


find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
# add_message_files(
#   FILES
# )
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

add_executable(octo_avoid src/main.cpp)
target_link_libraries(octo_avoid ${OCTOMAP_LIBRARIES} ${CATKIN_LIBRARIES})
target_link_libraries(octo_avoid ${CATKIN_LIBRARIES})
target_link_libraries(octo_avoid ${roscpp_LIBRARIES})
add_executable(example src/simple_example.cpp)
target_link_libraries(example ${OCTOMAP_LIBRARIES})
target_link_libraries(example ${CATKIN_LIBRARIES})
target_link_libraries(example ${roscpp_LIBRARIES})
