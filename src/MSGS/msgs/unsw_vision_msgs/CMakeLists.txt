cmake_minimum_required(VERSION 3.0.2)
project(unsw_vision_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  vision_msgs
  message_generation
  sensor_msgs
  diagnostic_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
     ObjectDetection.msg
     BoundingBox.msg
     DetectionList.msg
     PersonDetection.msg
     BagPointDetection.msg
     PersonPointing.msg
     ObjectDetectionList.msg
     VisualVector.msg
     InteractedObject.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  setFrameRates.srv
  startTrackingPerson.srv
  stopTrackingPerson.srv
  lookup.srv
  Gender.srv
  RecogniseObjects.srv
  MemorisePerson.srv
  FindObjectFrame.srv
  # IdentifyPerson.srv
  ViewObjects.srv
  FindObjectFromVector.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  diagnostic_msgs
  sensor_msgs
  vision_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES
   #unsw_vision_msgs_test
   CATKIN_DEPENDS
     roscpp
     message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Test node, to force compilation of message header files
#add_library(unsw_vision_msgs_test src/test.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(unsw_vision_msgs_test vision_msgs_generate_messages)

## Specify libraries to link a library or executable target against
#target_link_libraries(unsw_vision_msgs_test ${catkin_LIBRARIES} )
