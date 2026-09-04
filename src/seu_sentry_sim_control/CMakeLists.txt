cmake_minimum_required(VERSION 3.8)
project(seu_sentry_sim_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(swerve_sim_controller_node src/swerve_sim_controller.cpp)
ament_target_dependencies(swerve_sim_controller_node
  geometry_msgs rclcpp sensor_msgs std_msgs
)

install(TARGETS swerve_sim_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
