cmake_minimum_required(VERSION 3.8)
project(launch_everything)

find_package(ament_cmake REQUIRED)

# Installiere die Launch-Dateien
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()