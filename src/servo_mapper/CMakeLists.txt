cmake_minimum_required(VERSION 3.8)
project(servo_mapper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)

add_executable(servo_mapper_node src/servo_mapper_node.cpp)
ament_target_dependencies(servo_mapper_node rclcpp sensor_msgs dynamixel_sdk_custom_interfaces)

# Hier ggf. DynamixelSDK und SCServo als externe Libs einbinden:
# target_link_libraries(servo_mapper_node ...)

install(TARGETS servo_mapper_node DESTINATION lib/${PROJECT_NAME})

ament_package() 