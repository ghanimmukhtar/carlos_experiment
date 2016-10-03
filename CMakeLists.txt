cmake_minimum_required(VERSION 2.8.3)
project(carlos_experiment)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  trajectory_msgs
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crustcrawler_cppkdl
#  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

# EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS system )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(carlos_experiment_light src/carlos_experiment_light.cpp)
target_link_libraries(carlos_experiment_light ${catkin_LIBRARIES})

add_executable(carlos_experiment_light_test src/carlos_experiment_light_test.cpp)
target_link_libraries(carlos_experiment_light_test ${catkin_LIBRARIES})
