cmake_minimum_required(VERSION 3.0.2)
project(split_belt_controller)

## Enable C++11/14 support
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quad_utils
)

## System dependencies (Boost is an example, add if needed)
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES split_belt_controller
  CATKIN_DEPENDS roscpp std_msgs quad_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library (if needed for future use)
add_library(${PROJECT_NAME}
  src/split_belt_controller.cpp
)

## Add dependencies for the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare the C++ executable
add_executable(${PROJECT_NAME}_node
  src/split_belt_controller_node.cpp
)

## Add dependencies for the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link the executable with necessary libraries
target_link_libraries(${PROJECT_NAME}_node
split_belt_controller
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Install the C++ executable
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest-based cpp test (optional)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_split_belt_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()