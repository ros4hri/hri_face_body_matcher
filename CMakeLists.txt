# Copyright 2023 PAL Robotics S.L.
# All Rights Reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

cmake_minimum_required(VERSION 3.1.0)
project(hri_face_body_matcher)

set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  hri
  hri_msgs
  roscpp
  sensor_msgs
)

find_package(dlib REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS hri hri_msgs roscpp sensor_msgs
  DEPENDS dlib
) 

###########
## Build ##
###########

include_directories(
  src
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/match.cpp src/face_body_matcher.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME match PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_node
  dlib::dlib
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_hri_face_body_matcher_default.launch)
  add_rostest(test/hri_face_body_matcher_high_confidence_scaling.test)
  add_rostest(test/hri_face_body_matcher_high_confidence_threshold.test)
endif()
