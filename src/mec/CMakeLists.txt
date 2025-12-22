cmake_minimum_required(VERSION 3.8)
project(mec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(mecanum_drive_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)




# Install launch/config/worlds files
install(DIRECTORY
  launch
  new_launch
  config
  worlds
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install models (URDF/Xacro and meshes)
install(DIRECTORY
  model
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# RViz config
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
    PROGRAMS
    scripts/driver_node.py
    scripts/pid_node.py
    scripts/encoder_node.py
    scripts/move_node.py
    scripts/lidar_filter.py 
    scripts/odom_node.py  
    scripts/imu.py   
    scripts/obstacle_Detection_Node.py    
    scripts/lane_detection_node.py
    scripts/mecanum_motion.py
    scripts/scenario_01.py 
    scripts/scenario_02.py       
    DESTINATION lib/${PROJECT_NAME}
)



add_library(mecanum_lateral_plugin SHARED src/plugins/mecanum_lateral_plugin.cpp)
ament_target_dependencies(mecanum_lateral_plugin rclcpp geometry_msgs)
target_link_libraries(mecanum_lateral_plugin ${GAZEBO_LIBRARIES})
install(TARGETS mecanum_lateral_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
include_directories(
  include
  src/plugins
  ${rclcpp_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_ROS_INCLUDE_DIRS}  # لو موجود
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
