cmake_minimum_required(VERSION 2.8.3)
project(jsk_2021_10_semi)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(
  OpenCV REQUIRED
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

