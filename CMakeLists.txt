cmake_minimum_required(VERSION 3.0.2)
project(data_transfer)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  data_transfer_msg
  roscpp
  sensor_msgs
  std_msgs
        )

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS darknet_ros_msgs data_transfer_msg roscpp sensor_msgs std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/data_transfer_node.cpp src/data_transfer_node.cpp include/data_transfer/data_transfer.h src/data_transfer.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
