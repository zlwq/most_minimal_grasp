cmake_minimum_required(VERSION 3.4)
project(franka_gazebo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 架构检测（可选）
execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCHITECTURE OUTPUT_STRIP_TRAILING_WHITESPACE)
if(${ARCHITECTURE} STREQUAL "armv7l")
  message(WARNING "franka_gazebo was never tested on ARM. Use at own risk!")
endif()

# ------------------- 依赖 -------------------
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
  gazebo_ros_control
  kdl_parser
  boost_sml
  control_toolbox
  controller_manager
  controller_interface
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  eigen_conversions
  urdf
  angles
  franka_hw
  franka_msgs
  franka_gripper
)

# Gazebo & Franka
find_package(gazebo REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

# ------------------- catkin 包配置 -------------------
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    gazebo_ros_control
    kdl_parser
    boost_sml
    controller_manager
    controller_interface
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    eigen_conversions
    urdf
    angles
    franka_hw
    franka_msgs
    franka_gripper
  LIBRARIES franka_hw_sim franka_gripper_sim
  DEPENDS Franka orocos_kdl gazebo
)

# ------------------- include & link -------------------
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

# ------------------- 库定义 -------------------
add_library(franka_hw_sim
  src/franka_hw_sim.cpp
  src/joint.cpp
  src/model_kdl.cpp
  src/controller_verifier.cpp
)
if (Franka_VERSION GREATER_EQUAL 0.9)
  target_compile_definitions(franka_hw_sim PUBLIC ENABLE_BASE_ACCELERATION)
endif()

target_link_libraries(franka_hw_sim
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(franka_hw_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(franka_gripper_sim
  src/franka_gripper_sim.cpp
)

target_link_libraries(franka_gripper_sim
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(franka_gripper_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ------------------- 安装 -------------------
install(
  FILES franka_hw_sim_plugin.xml franka_gripper_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS franka_hw_sim franka_gripper_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
