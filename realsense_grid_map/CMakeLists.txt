cmake_minimum_required(VERSION 3.0.2)
project(realsense_grid_map)

## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Find Eigen3
find_package(Eigen3 REQUIRED)

## Find PCL (Point Cloud Library)
find_package(PCL REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_conversions   # For ROS <-> PCL conversions
  pcl_ros           # ROS PCL wrapper
  grid_map_ros      # For grid map handling
  grid_map_msgs     # For grid map messages
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_conversions pcl_ros grid_map_ros grid_map_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  # Include PCL directories
)

add_executable(${PROJECT_NAME}_node src/realsense_grid_map_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME realsense_grid_map_node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}     # Link PCL libraries
  Eigen3::Eigen
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
