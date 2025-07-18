cmake_minimum_required(VERSION 3.8)
project(leap_node)

# Leap Motion SDK einbinden
set(LEAPSDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LeapSDK)
include_directories(${LEAPSDK_DIR}/include)
link_directories(${LEAPSDK_DIR}/lib/x64)

# ROS 2 Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

# Nachrichten generieren
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoneData.msg"
  "msg/FingerData.msg"
  "msg/HandData.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

#############################################
# Sample Node
add_executable(sample_node src/Sample.cpp)
ament_target_dependencies(sample_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  tf2
  visualization_msgs
)
target_include_directories(sample_node PUBLIC
  ${tf2_INCLUDE_DIRS}
)
target_link_libraries(sample_node Leap)


set_target_properties(sample_node PROPERTIES
  INSTALL_RPATH "${LEAPSDK_DIR}/lib/x64"
)

#############################################
# DOF Publisher Node
add_executable(dof_publisher_node src/DOF_publisher_node.cpp)
target_link_libraries(dof_publisher_node Leap)

target_include_directories(dof_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dof_publisher_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  tf2
  visualization_msgs
)

# <--- WICHTIG: Abhängigkeit zu den generierten Messages hinzufügen!
rosidl_target_interfaces(dof_publisher_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Installation
install(TARGETS
  sample_node
  dof_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
