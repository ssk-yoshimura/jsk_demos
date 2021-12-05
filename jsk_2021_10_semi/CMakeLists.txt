cmake_minimum_required(VERSION 2.8.3)
project(jsk_2021_10_semi)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  message_runtime
  actionlib
  actionlib_msgs
  cv_bridge
  jsk_recognition_msgs
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

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

