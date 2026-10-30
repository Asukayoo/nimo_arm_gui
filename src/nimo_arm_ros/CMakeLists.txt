cmake_minimum_required(VERSION 3.8)
project(nimo_arm_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nimo_arm_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 QUIET NO_MODULE)

# Link against host nimo_arm library (container path /home/mundane/nimo_arm).
set(NIMO_ARM_ROOT "" CACHE PATH "Path to nimo_arm source tree")
if(NOT NIMO_ARM_ROOT)
  if(DEFINED ENV{NIMO_ARM_ROOT})
    set(NIMO_ARM_ROOT $ENV{NIMO_ARM_ROOT})
  elseif(EXISTS "/home/mundane/nimo_arm")
    set(NIMO_ARM_ROOT "/home/mundane/nimo_arm")
  elseif(EXISTS "/home/nimo/V1_Hardware_Control/nimo_arm")
    set(NIMO_ARM_ROOT "/home/nimo/V1_Hardware_Control/nimo_arm")
  endif()
endif()

if(NOT NIMO_ARM_ROOT OR NOT EXISTS "${NIMO_ARM_ROOT}/include/nimo_arm/arm_control.hpp")
  message(FATAL_ERROR
    "nimo_arm not found. Set -DNIMO_ARM_ROOT=/path/to/nimo_arm "
    "(expected include/nimo_arm/arm_control.hpp).")
endif()

set(NIMO_ARM_BUILD "${NIMO_ARM_ROOT}/build" CACHE PATH "nimo_arm build dir with libnimo_arm.a")
if(NOT EXISTS "${NIMO_ARM_BUILD}/libnimo_arm.a")
  message(FATAL_ERROR "libnimo_arm.a not found under ${NIMO_ARM_BUILD}. Build nimo_arm first.")
endif()

add_executable(dual_arm_node
  src/dual_arm_main.cpp
  src/dual_arm_node_core.cpp
  src/dual_arm_streaming.cpp
  src/dual_arm_teleop.cpp
  src/dual_arm_motion.cpp
  src/dual_arm_waypoints.cpp
  src/dual_arm_record.cpp
  src/dual_arm_traj.cpp
  src/rt_thread.cpp
  src/run_diagnostics.cpp
  src/traj_store.cpp
)
target_include_directories(dual_arm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  "${NIMO_ARM_ROOT}/include"
)
if(NOT TARGET Eigen3::Eigen)
  target_include_directories(dual_arm_node PUBLIC /usr/include/eigen3)
endif()

target_compile_definitions(dual_arm_node PRIVATE
  NIMO_ARM_DEFAULT_CONFIG="${NIMO_ARM_ROOT}/config/arm.yaml"
)

# ament first (plain link signature), then other libs also plain.
ament_target_dependencies(dual_arm_node
  rclcpp rclcpp_action sensor_msgs geometry_msgs std_msgs std_srvs nimo_arm_msgs
)

set(RUCKIG_LIB "${NIMO_ARM_BUILD}/ruckig_part/libruckig_part.a")
set(_extra_libs
  "${NIMO_ARM_BUILD}/libnimo_arm.a"
  yaml-cpp
  Threads::Threads
)
if(TARGET Eigen3::Eigen)
  list(APPEND _extra_libs Eigen3::Eigen)
endif()
if(EXISTS "${RUCKIG_LIB}")
  list(APPEND _extra_libs "${RUCKIG_LIB}")
endif()

set(NIMO_ARM_SDK_ROOT
  "${NIMO_ARM_ROOT}/../tj_control-main/TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK"
  CACHE PATH "TJ FX robot SDK root")
find_library(NIMO_ARM_MARVIN_SDK MarvinSDK
  PATHS
    "${NIMO_ARM_SDK_ROOT}/contrlSDK"
    "${NIMO_ARM_SDK_ROOT}/DEMO_C++"
  NO_DEFAULT_PATH)
if(NIMO_ARM_MARVIN_SDK)
  link_directories(
    "${NIMO_ARM_SDK_ROOT}/contrlSDK"
    "${NIMO_ARM_SDK_ROOT}/DEMO_C++")
  list(APPEND _extra_libs ${NIMO_ARM_MARVIN_SDK})
endif()

target_link_libraries(dual_arm_node ${_extra_libs})

install(TARGETS dual_arm_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
