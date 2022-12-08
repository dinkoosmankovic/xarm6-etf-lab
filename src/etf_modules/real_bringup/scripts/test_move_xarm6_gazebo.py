#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand as GripperCommandAction
from control_msgs.msg import GripperCommand
from rclpy.action import ActionClient
import numpy as np

class TestJointTrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_publisher_node')
        trajectory_topic = '/xarm6_traj_controller/joint_trajectory'
        gripper_topic = '/xarm_gripper/gripper_action'
        self.trajectory_publisher = \
            self.create_publisher(JointTrajectory, trajectory_topic, 10)

        self.gripper_client = ActionClient(self, GripperCommandAction, gripper_topic)

        self.pick = True

        timer_period = 7.0
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

    def pick_procedure(self):
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
        return trajectory

    def place_procedure(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        joint_point3 = JointTrajectoryPoint()
        joint_point3.positions = [
            -0.5,
            0.0,
            -0.6,
            np.pi,
            np.pi / 2,
            0.0,
            ]
        joint_point3.time_from_start = Duration(sec=1)
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

        joint_point4.time_from_start = Duration(sec=2)
        trajectory.points.append(joint_point4)
        return trajectory

    def timer_callback(self):
        if (self.pick):
            trajectory = self.pick_procedure()
            self.trajectory_publisher.publish(trajectory)
            ## GRIPPER ##########################################

            gripper_command = GripperCommandAction.Goal()
            gripper_command.command = GripperCommand(position=0.1, max_effort=5.0)
            self.gripper_client.send_goal_async(gripper_command)
            self.pick = False

        else:
            trajectory = self.place_procedure()
            self.trajectory_publisher.publish(trajectory)
            ## GRIPPER ##########################################

            gripper_command = GripperCommandAction.Goal()
            gripper_command.command = GripperCommand(position=0.8, max_effort=5.0)
            self.gripper_client.send_goal_async(gripper_command)
            self.pick = True
        


def main(args=None):
    rclpy.init(args=args)
    joint_publisher = TestJointTrajectoryNode()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
