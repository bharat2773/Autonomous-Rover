cmake_minimum_required(VERSION 3.5)
project(rover_description)

find_package(ament_cmake REQUIRED)


# Add after find_package commands

install(DIRECTORY
  worlds 
  launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
