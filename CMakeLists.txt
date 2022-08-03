cmake_minimum_required(VERSION 3.0.2)
project(Data_Transfer)

find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp sensor_msgs std_msgs)

find_package(OpenCV REQUIRED)

add_message_files(FILES data_transfer.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

#catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} src/Data_Transfer_node.cpp src/Data_Transfer.cpp include/Data_Transfer/Data_Transfer.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})