cmake_minimum_required(VERSION 3.5)

project(ros2_bot)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH "/usr/local/lib/cmake;${CMAKE_MODULE_PATH}")
find_package(raspicam REQUIRED)

#######################################################################################
# Hello World node

set(
 HELLO_NODE_SRC
 ./src/hello_world/hello.cpp
)

add_executable(hello_world ${HELLO_NODE_SRC})
ament_target_dependencies(hello_world
  "std_msgs"
  "rclcpp"
  "sensor_msgs"
)
install(TARGETS hello_world DESTINATION bin)

#######################################################################################
# Camera node

set(
 CAMERA_NODE_SRC
  ./src/camera_node/camera_node.cpp
)

add_executable(camera_node ${CAMERA_NODE_SRC})
ament_target_dependencies(camera_node
  "std_msgs"
  "rclcpp"
  "raspicam"
  "sensor_msgs"
  "OpenCV"
)
target_link_libraries(camera_node raspicam_cv raspicam)
install(TARGETS camera_node DESTINATION bin)

#######################################################################################

ament_package()
