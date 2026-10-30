cmake_minimum_required(VERSION 3.8)
project(nimo_arm_description)

find_package(ament_cmake REQUIRED)

# Meshes: symlink or copy under meshes/ (from tj_control Marvin M6 URDF).
install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
