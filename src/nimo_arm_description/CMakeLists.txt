cmake_minimum_required(VERSION 3.8)
project(nimo_arm_description)

find_package(ament_cmake REQUIRED)

# Meshes: Marvin M3-S-CCS-615 dual-arm STLs under meshes/ (vendored from tj_control).
install(DIRECTORY urdf launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
