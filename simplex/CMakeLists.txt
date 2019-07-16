cmake_minimum_required(VERSION 2.8.3)
project(simplex)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  cmake_modules
  tf
)
find_package(Eigen REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   base_msg.msg
#   Message2.msg
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simplex
   CATKIN_DEPENDS roscpp std_msgs  message_runtime
   DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
add_library(Simplex
  src/Simplex.cpp  
)
target_link_libraries(Simplex
  ${catkin_LIBRARIES}
)

add_definitions(${EIGEN_DEFINITIONS})

add_executable(main src/main.cpp)
target_link_libraries(main   
    ${catkin_LIBRARIES}  Simplex)
add_dependencies(main ${catkin_EXPORTED_TARGETS})
