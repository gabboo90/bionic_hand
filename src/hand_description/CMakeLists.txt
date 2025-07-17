cmake_minimum_required(VERSION 3.8)
project(hand_description)

find_package(ament_cmake REQUIRED)

# Installiere die URDF-Dateien
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Installiere die Meshes
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Installiere die Launch-Dateien
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()