.PHONY: clean build dependencies sim real rviz

dependencies:
	rosdep install --from-paths src --ignore-src -r -y

clean:
	rm -r ./build/ ./install/ ./log/

build:
	colcon build --symlink-install --cmake-args " -DCMAKE_BUILD_TYPE=RelWithDebInfo"

source:
	source /opt/ros/humble/setup.bash
	source install/setup.bash
	source /usr/share/gazebo/setup.bash

sim:
	# make build
	ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py
