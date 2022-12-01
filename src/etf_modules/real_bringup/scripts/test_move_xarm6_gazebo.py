#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class TestJointTrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_publisher_node')
        trajectory_topic = '/xarm6_traj_controller/joint_trajectory'
        gripper_topic = '/xarm_gripper_traj_controller/joint_trajectory'
        self.trajectory_publisher = \
            self.create_publisher(JointTrajectory, trajectory_topic, 10)

        self.gripper_publisher = \
            self.create_publisher(JointTrajectory, gripper_topic, 10)    
        timer_period = 5.0
        self.timer = self.create_timer(timer_period,
                self.timer_callback)
        self.joints = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            ]

        self.gripper_joint = ['drive_joint']

    def timer_callback(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        joint_point1 = JointTrajectoryPoint()
        joint_point1.positions = [
            0.0,
            0.0,
            0.0,
            np.pi,
            np.pi / 2,
            0.0,
            ]

            
        joint_point1.time_from_start = Duration(sec=1)
        trajectory.points.append(joint_point1)

        joint_point2 = JointTrajectoryPoint()
        joint_point2.positions = [
            0.5,
            0.0,
            -0.6,
            np.pi,
            np.pi / 2,
            0.0,
            ]
        joint_point2.time_from_start = Duration(sec=2)
        trajectory.points.append(joint_point2)

        joint_point3 = JointTrajectoryPoint()
        joint_point3.positions = [
            -0.5,
            0.0,
            -0.6,
            np.pi,
            np.pi / 2,
            0.0,
            ]
        joint_point3.time_from_start = Duration(sec=3)
        trajectory.points.append(joint_point3)

        joint_point4 = JointTrajectoryPoint()
        joint_point4.positions = [
            0.0,
            0.0,
            0.0,
            np.pi,
            np.pi / 2,
            0.0,
            ]

        joint_point4.time_from_start = Duration(sec=4)
        trajectory.points.append(joint_point4)

        ## GRIPPER ##########################################

        gripper_trajectory = JointTrajectory()
        gripper_trajectory.joint_names = self.gripper_joint
        gripper_point1 = JointTrajectoryPoint()
        gripper_point1.positions = [0.0]
            
        gripper_point1.time_from_start = Duration(sec=1)
        gripper_trajectory.points.append(gripper_point1)

        gripper_point2 = JointTrajectoryPoint()
        gripper_point2.positions = [0.0]

        gripper_point2.time_from_start = Duration(sec=2)
        gripper_trajectory.points.append(gripper_point2)

        gripper_point3 = JointTrajectoryPoint()
        gripper_point3.positions = [0.85]

        gripper_point3.time_from_start = Duration(sec=3)
        gripper_trajectory.points.append(gripper_point3)

        gripper_point4 = JointTrajectoryPoint()
        gripper_point4.positions = [0.0]

        gripper_point4.time_from_start = Duration(sec=4)
        gripper_trajectory.points.append(gripper_point4)

        #####################################################

        self.trajectory_publisher.publish(trajectory)
        self.gripper_publisher.publish(gripper_trajectory)


def main(args=None):
    rclpy.init(args=args)
    joint_publisher = TestJointTrajectoryNode()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
