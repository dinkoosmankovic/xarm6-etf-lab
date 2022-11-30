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


    return LaunchDescription([
        camera_left_node,
        camera_right_node
    ])
