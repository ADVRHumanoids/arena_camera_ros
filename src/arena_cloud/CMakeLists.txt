cmake_minimum_required(VERSION 3.0)
project(arena_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  eigen_conversions
  message_generation
  geometry_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_thesis_gap
  CATKIN_DEPENDS message_runtime pcl_conversions pcl_ros roscpp geometry_msgs sensor_msgs std_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#   include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_executable(arena_cloud src/arena_cloud.cpp)
add_dependencies(arena_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arena_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS arena_cloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/
#   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )

# install(DIRECTORY config/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#     )
