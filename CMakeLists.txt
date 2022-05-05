cmake_minimum_required(VERSION 3.0.2)
project(edge_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge geometry_msgs image_transport sensor_msgs
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# add_library(img 
#   src/image/ImageConverter.cpp
# )

# target_link_libraries(img ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/image/ImageConverter.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  # img
)
