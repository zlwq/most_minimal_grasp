cmake_minimum_required(VERSION 3.0.2)
project(banana_grasp_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge 
  message_filters
  franka_gripper
  geometry_msgs
  actionlib
  std_srvs
  moveit_ros_planning_interface 
  moveit_visual_tools      
  tf2
  tf2_ros
  moveit_core 
  gazebo_msgs
  gazebo_ros
)
find_package(OpenCV REQUIRED) 
find_package(FLTK REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge message_filters image_transport
)
# 显式指定 FLTK 的包含目录和库目录
set(FLTK_INCLUDE_DIRS /usr/include/FL)
set(FLTK_LIBRARIES /usr/lib/x86_64-linux-gnu/libfltk.so) # 根据实际路径调整
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(banana_grasp src/banana_grasp.cpp)
target_link_libraries(banana_grasp
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${FLTK_LIBRARIES}
)