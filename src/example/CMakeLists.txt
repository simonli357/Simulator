cmake_minimum_required(VERSION 3.0.2)
project(example)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  utils
  roscpp
  sensor_msgs
  tf2_geometry_msgs
  cv_bridge
)

## Find OpenCV
find_package(OpenCV REQUIRED)

## Find Intel RealSense
find_package(realsense2 REQUIRED)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    tf2_geometry_msgs
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

add_executable(serial_imu_publisher src/serial_imu_reader.cpp)
target_link_libraries(serial_imu_publisher ${catkin_LIBRARIES})
add_executable(serial_write src/serial_write.cpp)
target_link_libraries(serial_write ${catkin_LIBRARIES})
add_executable(realsense_read src/realsense_read.cpp)
target_link_libraries(realsense_read ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
${realsense2_LIBRARY})

## Install
install(TARGETS serial_imu_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS serial_write
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS realsense_read
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)