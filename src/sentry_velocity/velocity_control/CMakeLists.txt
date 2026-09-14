cmake_minimum_required(VERSION 3.8)
project(velocity_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(robots_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# 创建控制器库
add_library(velocity_controller SHARED
  src/velocity_controller.cpp
)

target_include_directories(velocity_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(velocity_controller
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  robots_msgs
  sensor_msgs
  tf2_ros
  control_toolbox
)

pluginlib_export_plugin_description_file(controller_interface velocity_controller_plugin.xml)


# 安装库
install(TARGETS velocity_controller
  EXPORT export_velocity_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Add pp2->livox converter executable for simulation-side
add_executable(pp2_to_livox_node src/pp2_to_livox.cpp)
target_include_directories(pp2_to_livox_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(pp2_to_livox_node rclcpp sensor_msgs std_msgs)
ament_target_dependencies(pp2_to_livox_node livox_ros_driver2)

install(TARGETS pp2_to_livox_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(imu_alias_node src/imu_alias.cpp)
target_include_directories(imu_alias_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(imu_alias_node rclcpp sensor_msgs)

install(TARGETS imu_alias_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(wheel_sim_adapter_node src/wheel_sim_adapter.cpp)
ament_target_dependencies(wheel_sim_adapter_node
  geometry_msgs nav_msgs rclcpp robots_msgs sensor_msgs std_msgs
)

install(TARGETS wheel_sim_adapter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(cmd_vel_mux_node src/cmd_vel_mux.cpp)
ament_target_dependencies(cmd_vel_mux_node
  geometry_msgs rclcpp std_msgs
)

install(TARGETS cmd_vel_mux_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(keyboard_sim_adapter_node src/keyboard_sim_adapter.cpp)
ament_target_dependencies(keyboard_sim_adapter_node
  geometry_msgs rclcpp sensor_msgs std_msgs
)

install(TARGETS keyboard_sim_adapter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(control_mode_once_node src/control_mode_once.cpp)
ament_target_dependencies(control_mode_once_node rclcpp std_msgs)

install(TARGETS control_mode_once_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(keyboard_teleop_node src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop_node geometry_msgs rclcpp std_msgs)

install(TARGETS keyboard_teleop_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY ../gazeboworld/
  DESTINATION share/${PROJECT_NAME}/gazeboworld
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_velocity_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  robots_msgs
  sensor_msgs
  tf2_ros
  control_toolbox
)
ament_package()
