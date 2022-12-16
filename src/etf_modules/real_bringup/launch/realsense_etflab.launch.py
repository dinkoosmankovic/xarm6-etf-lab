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
            name="left_transform",
            executable = "static_transform_publisher",
            arguments = ["-1.0181", "-0.4781", "0.5363", "0.7991", "0.3533", "0", \
                        "aruco_marker", "camera_left_link"]
	)

    tf_node_aruco_right_camera = Node(package = "tf2_ros", 
            name="right_transform",
            executable = "static_transform_publisher",
            arguments = ["-0.3126", "0.1587", "0.9515", "2.1487", "0.8906", "0.5158", \
                        "camera_right_color_optical_frame", "aruco_marker_from_right"]
	)

    tf_node_arucos_tf = Node(package = "tf2_ros", 
            name="arucos_tf",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", \
                        "aruco_marker_from_right", "aruco_marker"]
	)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        #=['-d', default_rviz_config_path],
        #parameters=[{'use_sim_time': use_sim_time}]
    )


    return LaunchDescription([
        camera_left_node,
        camera_right_node,
        #tf_node_world_link_base,
        #tf_node_world_aruco,
        tf_node_aruco_left_camera,
        #tf_node_aruco_right_camera,
        #tf_node_arucos_tf,
        rviz_node
    ])
