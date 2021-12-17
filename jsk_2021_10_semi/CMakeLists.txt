cmake_minimum_required(VERSION 2.8.3)
project(jsk_2021_10_semi)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  message_generation
  message_runtime
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  jsk_recognition_msgs
)

find_package(
  OpenCV REQUIRED
)

add_action_files(
  FILES
  Yoshimura.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  sensor_msgs
  jsk_recognition_msgs
)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable(
 cv_bridge_tutorial src/cv_bridge_tutorial.cpp
)

target_link_libraries(
 cv_bridge_tutorial
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

