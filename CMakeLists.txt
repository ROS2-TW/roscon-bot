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
macro(targets)

if(NOT target_suffix STREQUAL "")
  get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
endif()

# Hello World node
set(
 HELLO_NODE_SRC
 ./src/hello_world/hello.cpp
)

add_executable(hello_world${target_suffix} ${HELLO_NODE_SRC})
ament_target_dependencies(hello_world${target_suffix} "rclcpp${target_suffix}")
ament_target_dependencies(hello_world${target_suffix} "std_msgs")
ament_target_dependencies(hello_world${target_suffix} "sensor_msgs")
install(TARGETS hello_world${target_suffix} DESTINATION bin)

# Camera node
set(
 CAMERA_NODE_SRC
  ./src/camera_node/camera_node.cpp
)

set(
  CAMERA_NODE_INC
  ./src/camera_node
)

include_directories(${CAMERA_NODE_INC})
add_executable(camera_node${target_suffix} ${CAMERA_NODE_SRC})
ament_target_dependencies(camera_node${target_suffix} "rclcpp${target_suffix}")
ament_target_dependencies(camera_node${target_suffix} "std_msgs")
ament_target_dependencies(camera_node${target_suffix} "raspicam")
ament_target_dependencies(camera_node${target_suffix} "sensor_msgs")
ament_target_dependencies(camera_node${target_suffix} "OpenCV")
target_link_libraries(camera_node${target_suffix} raspicam_cv raspicam)
install(TARGETS camera_node${target_suffix} DESTINATION bin)

endmacro()
#######################################################################################

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

ament_package()
