.PHONY: clean build dependencies source-dirs sim

dependencies:
	rosdep install --from-paths src --ignore-src -r -y

clean:
	rm -r ./build/ ./install/ ./log/

build:
	colcon build --symlink-install --cmake-args " -DCMAKE_BUILD_TYPE=RelWithDebInfo"

source-dirs:
	/bin/bash /root/etf-xarm-lab/source-dirs.bash

sim:
	# make build
	ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py
