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
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(control_msgs REQUIRED)
find_package(fcl 0.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rpmpl_library REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
  ${RPMPL_LIBRARY_INCLUDE_DIRS}
  )

add_executable(test_move_xarm6 src/test_move_xarm6.cpp)
target_compile_features(test_move_xarm6 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_move_xarm6 PUBLIC 
  rclcpp 
  trajectory_msgs
)

add_executable(test_read_octomap src/test_read_octomap.cpp)
ament_target_dependencies(test_read_octomap 
  rclcpp 
  octomap_msgs 
  octomap 
  fcl 
  Eigen3
)

set(TARGET_DEPENDENCIES 
  rclcpp 
  trajectory_msgs 
  octomap_msgs 
  visualization_msgs 
  control_msgs 
  sensor_msgs
  octomap 
  fcl 
  Eigen3 
  pcl_conversions 
  PCL
)

add_executable(test_planners src/test_planners.cpp)
target_compile_features(test_planners PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_planners PUBLIC 
  ${RPMPL_LIBRARY} 
  ${RPMPL_LIBRARIES}
)
ament_target_dependencies(test_planners PUBLIC 
  ${TARGET_DEPENDENCIES}
)

add_library(RealTimeDRGBTConnect src/RealTimeDRGBTConnect.cpp)
target_compile_features(RealTimeDRGBTConnect PUBLIC c_std_99 cxx_std_17)
target_link_libraries(RealTimeDRGBTConnect PUBLIC 
  ${RPMPL_LIBRARY} 
  ${RPMPL_LIBRARIES}
)

add_executable(test_RealTimeDRGBTConnect src/test_RealTimeDRGBTConnect.cpp)
target_compile_features(test_RealTimeDRGBTConnect PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_RealTimeDRGBTConnect PUBLIC 
  RealTimeDRGBTConnect
  ${RPMPL_LIBRARY} 
  ${RPMPL_LIBRARIES}
)
ament_target_dependencies(test_RealTimeDRGBTConnect PUBLIC 
  ${TARGET_DEPENDENCIES}
)


add_executable(test_sim_bringup_node src/test_sim_bringup_node.cpp)
target_include_directories(test_sim_bringup_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_sim_bringup_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  test_sim_bringup_node 
  test_move_xarm6 
  test_read_octomap
  test_planners
  test_RealTimeDRGBTConnect
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
  src/test_move_xarm6.py
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
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(control_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(PCL)

ament_package()