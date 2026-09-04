cmake_minimum_required(VERSION 3.8)
project(seu_sentry_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")
ament_package()
