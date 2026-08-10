cmake_minimum_required(VERSION 3.8)
project(qcar_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY urdf launch worlds models rviz scripts config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/qcar_teleop_twist.py
  scripts/qcar_relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
