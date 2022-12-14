cmake_minimum_required(VERSION 3.8)
project(perception_etflab)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED common filters segmentation sample_consensus)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  	include
  	${EIGEN3_INCLUDE_DIR}
)

add_executable(pointcloud_combiner_node src/pointcloud_combiner.cpp)
target_link_libraries(pointcloud_combiner_node 
	${PCL_LIBRARIES}
)

add_executable(object_segmentation_node src/object_segmentation_node.cpp src/object_segmentation.cpp)
target_link_libraries(object_segmentation_node 
	${PCL_LIBRARIES}
)

ament_target_dependencies(pointcloud_combiner_node
  	rclcpp 
  	sensor_msgs
  	geometry_msgs
  	PCL
  	pcl_conversions
  	tf2_sensor_msgs
  	tf2_eigen
)

ament_target_dependencies(object_segmentation_node
  	rclcpp 
  	sensor_msgs
  	geometry_msgs
  	PCL
  	pcl_conversions
  	tf2_sensor_msgs
  	tf2_eigen
)

install(TARGETS
  	pointcloud_combiner_node
  	object_segmentation_node
  	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  	find_package(ament_lint_auto REQUIRED)
  	# the following line skips the linter which checks for copyrights
  	# uncomment the line when a copyright and license is not present in all source files
  	#set(ament_cmake_copyright_FOUND TRUE)
  	# the following line skips cpplint (only works in a git repo)
  	# uncomment the line when this package is not in a git repo
  	#set(ament_cmake_cpplint_FOUND TRUE)
  	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
