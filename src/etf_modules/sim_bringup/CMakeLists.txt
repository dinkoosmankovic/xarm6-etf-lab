cmake_minimum_required(VERSION 3.8)
project(sim_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(fcl 0.7 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(librpmpl_library REQUIRED)
include_directories(${librpmpl_library_INCLUDE_DIRS})



add_executable(test_move_xarm6_gazebo scripts/test_move_xarm6_gazebo.cpp)
ament_target_dependencies(test_move_xarm6_gazebo rclcpp trajectory_msgs)

add_executable(test_read_octree scripts/test_read_octree.cpp)
ament_target_dependencies(test_read_octree rclcpp octomap_msgs fcl Eigen3)

add_executable(run_RPMPLv2 scripts/run_RPMPLv2.cpp)
ament_target_dependencies(run_RPMPLv2 rclcpp)
target_link_libraries(run_RPMPLv2 ${librpmpl_library_SHARED_LIBRARY})

add_executable(sim_bringup_node src/sim_bringup_node.cpp)
target_include_directories(sim_bringup_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sim_bringup_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  sim_bringup_node 
  test_move_xarm6_gazebo 
  test_read_octree
  run_RPMPLv2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  world
  launch
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/test_move_xarm6_gazebo.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(trajectory_msgs)
ament_export_dependencies(octomap_msgs)
ament_export_dependencies(fcl 0.7)
ament_export_dependencies(Eigen3)
ament_export_dependencies(librpmpl_library)

ament_package()