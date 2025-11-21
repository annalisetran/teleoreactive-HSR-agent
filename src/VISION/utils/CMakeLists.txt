cmake_minimum_required(VERSION 3.0.2)
project(unsw_vision_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  unsw_vision_msgs
  message_generation
  diagnostic_msgs
  std_msgs
  pcl_ros
  tf
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(GFlags)
find_package(Glog)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES unsw_vision_utils
  CATKIN_DEPENDS
    roscpp
    std_msgs unsw_vision_msgs message_runtime tf2_ros
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(unsw_vision_utils
  src/PositionGenerator.cpp
)

add_dependencies(unsw_vision_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(PositionGenerator
  src/PositionGenerator.cpp
)

add_executable(PlaneDetector
  src/PlaneDetector.cpp
)

target_link_libraries(PositionGenerator
 ${catkin_LIBRARIES}
 ${OpenCV_LIBS}
 unsw_vision_utils
)

target_link_libraries(PlaneDetector
 ${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)

add_executable(republisher
  src/republisher.cpp
)

target_link_libraries(republisher
  ${catkin_LIBRARIES}
)

# target_link_libraries(testNode
#  ${catkin_LIBRARIES}
# )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS
  src/BagHandleDetector.py
  src/PersonPointingDetector.py
  src/BagScanner.py
  src/EmptySeatDetector.py
  src/ObjectFrames.py
  src/ObjectFramesDebug.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_unsw_vision_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
