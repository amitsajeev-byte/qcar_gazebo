cmake_minimum_required(VERSION 3.8)
project(qcar_initial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# --- custom MPPI critic plugin (src/critics/) - see CHANGELOG.md 2026-07-18 (9) ---
# xtensor/xsimd defines below MUST match nav2_mppi_controller's own CMakeLists.txt exactly
# (github.com/ros-navigation/navigation2, humble branch) - xtensor's container layout/alignment
# changes depending on whether XTENSOR_USE_XSIMD is defined at compile time, so a mismatch here
# is a silent ABI mismatch: xt::xtensor<float,N> objects built by nav2's precompiled binary and
# by this plugin would disagree on memory layout despite being "the same" C++ type, corrupting
# memory on any vectorized operation (e.g. xt::pow) crossing the plugin boundary via CriticData.
# Confirmed live: without these defines, this plugin loaded and ran fine right up until any
# xtensor math heavier than a scalar += touched data.costs, then segfaulted the whole
# nav2_container process - see CHANGELOG.md 2026-07-18 (9) for the full isolation process.
add_definitions(-DXTENSOR_ENABLE_XSIMD)
add_definitions(-DXTENSOR_USE_XSIMD)

find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)

# DISABLED WHOLESALE 2026-09-02: this package's qcar_critics library (early_commit_critic.cpp)
# registered mppi::critics::EarlyCommitCritic into the SAME global pluginlib namespace as
# qcar_updated's own (and qcar_ackermann's) identically-named critic - nav2_mppi_controller's
# CriticManager::getFullName() hardcodes the "mppi::critics::" prefix, so there is no way to
# namespace these apart at the C++ level. A live qcar_updated launch confirmed the plugin actually
# resolved was a stale copy of this critic (still active_path_points:15, predating even the
# 2026-07-19 fix, let alone the 2026-09-02 max_lead_distance redesign) instead of qcar_updated's
# own rebuilt one - meaning qcar_updated's changes to this critic had zero effect on live testing
# until this was found. qcar_updated is now the only actively developed package - removing this
# target entirely rather than just its plugin export. See CHANGELOG.md 2026-09-02.
#
# add_library(qcar_critics SHARED
#   src/critics/early_commit_critic.cpp
# )
# target_include_directories(qcar_critics PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(qcar_critics
#   nav2_mppi_controller
#   nav2_costmap_2d
#   rclcpp
#   rclcpp_lifecycle
#   pluginlib
#   xtensor
# )
# target_link_libraries(qcar_critics xtensor::optimize xtensor::use_xsimd)
# pluginlib_export_plugin_description_file(nav2_mppi_controller qcar_critics.xml)
#
# install(TARGETS qcar_critics
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
# install(DIRECTORY include/
#   DESTINATION include
# )
# --- end custom MPPI critic plugin ---

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY urdf launch worlds models config rviz scripts maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/qcar_teleop_twist.py
  DESTINATION lib/${PROJECT_NAME}
)

# ament_export_include_directories/libraries/dependencies for the disabled qcar_critics target
# above were removed 2026-09-02 alongside it (see that comment block).

ament_package()
