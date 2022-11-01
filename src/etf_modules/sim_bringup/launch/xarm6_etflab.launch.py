#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, ETFSA
# All rights reserved.
#
# Author: Dinko Osmankovic <dinko.osmankovic@etf.unsa.ba>

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, Command, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    
    camera_left_x = LaunchConfiguration('camera_left_x', default=2)
    camera_left_y = LaunchConfiguration('camera_left_y', default=-1)
    camera_left_z = LaunchConfiguration('camera_left_z', default=0.6)
    camera_left_R = LaunchConfiguration('camera_left_R', default=0)
    camera_left_P = LaunchConfiguration('camera_left_P', default=0.15)
    camera_left_Y = LaunchConfiguration('camera_left_Y', default=2.8)
    
    camera_right_x = LaunchConfiguration('camera_right_x', default=2)
    camera_right_y = LaunchConfiguration('camera_right_y', default=1)
    camera_right_z = LaunchConfiguration('camera_right_z', default=0.6)
    camera_right_R = LaunchConfiguration('camera_right_R', default=0)
    camera_right_P = LaunchConfiguration('camera_right_P', default=0.15)
    camera_right_Y = LaunchConfiguration('camera_right_Y', default=3.5)
    
    # robot gazebo launch
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/_robot_spawn.launch.py']),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': '6',
            'robot_type': 'xarm',
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
            'camera_left_x': camera_left_x,
            'camera_left_y': camera_left_y,
            'camera_left_z': camera_left_z,
            'camera_left_R': camera_left_R,
            'camera_left_P': camera_left_P,
            'camera_left_Y': camera_left_Y,
            'camera_right_x': camera_right_x,
            'camera_right_y': camera_right_y,
            'camera_right_z': camera_right_z,
            'camera_right_R': camera_right_R,
            'camera_right_P': camera_right_P,
            'camera_right_Y': camera_right_Y,
        }.items(),
    )
        
    return LaunchDescription([
    	robot_gazebo_launch
    ])