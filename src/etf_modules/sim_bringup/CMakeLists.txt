cmake_minimum_required(VERSION 3.8)
project(sim_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(fcl 0.7 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(rpmpl_library REQUIRED)

add_executable(test_move_xarm6_gazebo scripts/test_move_xarm6_gazebo.cpp)
target_compile_features(test_move_xarm6_gazebo PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_move_xarm6_gazebo PUBLIC ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})
ament_target_dependencies(test_move_xarm6_gazebo PUBLIC rclcpp trajectory_msgs)

add_executable(test_read_octree scripts/test_read_octree.cpp)
ament_target_dependencies(test_read_octree rclcpp octomap_msgs fcl Eigen3)

add_executable(test_planners scripts/test_planners.cpp)
target_compile_features(test_planners PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_planners PUBLIC ${RPMPL_LIBRARY} ${RPMPL_LIBRARIES})
ament_target_dependencies(test_planners PUBLIC rclcpp)

add_executable(sim_bringup_node src/sim_bringup_node.cpp)
target_include_directories(sim_bringup_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sim_bringup_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  sim_bringup_node 
  test_move_xarm6_gazebo 
  test_read_octree
  test_planners
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

ament_package()