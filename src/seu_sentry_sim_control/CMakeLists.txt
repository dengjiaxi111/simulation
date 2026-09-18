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
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2-register REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(swerve_sim_controller_node src/swerve_sim_controller.cpp)
ament_target_dependencies(swerve_sim_controller_node
  geometry_msgs rclcpp sensor_msgs std_msgs
)

install(TARGETS swerve_sim_controller_node DESTINATION lib/${PROJECT_NAME})
add_library(startup_pose_lock SHARED src/startup_pose_lock.cpp)
target_link_libraries(startup_pose_lock
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
)
install(TARGETS startup_pose_lock LIBRARY DESTINATION lib)
add_library(sentry_motor_controller SHARED src/sentry_motor_controller.cpp)
target_link_libraries(sentry_motor_controller gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13)
install(TARGETS sentry_motor_controller LIBRARY DESTINATION lib)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
