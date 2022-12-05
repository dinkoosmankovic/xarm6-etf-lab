#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, ETFSA
# All rights reserved.
#
# Author: Dinko Osmankovic <dinko.osmankovic@etf.unsa.ba>

# ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=D400 device_type2:=l5. device_type1:=d4..

import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
    

def generate_launch_description():
    camera_left_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            #'initial_reset': 'true',
            'camera_name': 'camera_left',
            'serial_no': "_036222070643",
            'pointcloud.enable': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',

        }.items(),
    )

    camera_right_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            #'initial_reset': 'true',
            'camera_name': 'camera_right',
            'serial_no': "_036522072967",
            'pointcloud.enable': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',

        }.items(),
    )

    tf_node_world_link_base = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0.5", "0", "0", "0", "0", "world", "link_base"]
    )

    tf_node_world_aruco = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0.5", "0", "0", "0", "link_base", "aruco_marker"]
    )

    tf_node_aruco_left_camera = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            #arguments = ["0.48", "0.12", "1.15", "-2.0382", "0.6070", "2.8267", "aruco_marker", "camera_left_link"]
            arguments = ["0.48", "0.12", "1.15", "0.771", "0.0", "0", "aruco_marker", "camera_left_link"]
            #arguments = ["1", "-1", "0.5", "0.771", "0", "-1.771", "link_base", "camera_left"]
	)


    return LaunchDescription([
        camera_left_node,
        camera_right_node,
        tf_node_world_link_base,
        tf_node_world_aruco,
        tf_node_aruco_left_camera,
    ])
