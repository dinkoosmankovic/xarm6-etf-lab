cmake_minimum_required(VERSION 3.8)
project(aruco_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include 
  ${tf2_ros_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS} 
  ${sensor_msgs_INCLUDE_DIRS} 
  ${geometry_msgs_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR} 
)

add_executable(aruco_calibration src/aruco_calibration.cpp)
target_include_directories(aruco_calibration PRIVATE include/aruco_calibration)
target_link_libraries(aruco_calibration tf2)
ament_target_dependencies(
		aruco_calibration 
		rclcpp
		sensor_msgs
		OpenCV
		cv_bridge
		geometry_msgs
		tf2
		tf2_ros
)

install(TARGETS
    aruco_calibration
    DESTINATION lib/${PROJECT_NAME})


ament_package()
