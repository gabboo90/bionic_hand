cmake_minimum_required(VERSION 3.8)
project(servo_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)

# SCServo Library erstellen
file(GLOB SCServo_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
# Entferne die main node datei aus der library
list(REMOVE_ITEM SCServo_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/servo_mapper_node.cpp)

file(GLOB SCServo_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
)

# Static library für SCServo
add_library(SCServo STATIC ${SCServo_SOURCES} ${SCServo_HEADERS})
target_include_directories(SCServo PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Main executable
add_executable(servo_mapper_node src/servo_mapper_node.cpp)

target_include_directories(servo_mapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# ROS2 dependencies
ament_target_dependencies(servo_mapper_node 
  rclcpp 
  sensor_msgs 
  dynamixel_sdk_custom_interfaces
)

# Link SCServo library
target_link_libraries(servo_mapper_node SCServo)

# Install
install(TARGETS servo_mapper_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()