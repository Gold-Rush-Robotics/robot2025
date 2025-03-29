#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose, Vector3, Point
from std_msgs.msg import Bool

from state_actions.action import Action

# This action is meant to take care of placing the April tags in the beginning

class DriveToGap(Action):
    def __init__(self, name:str=None):
        super().__init__()
        
        if name:
            self.name = name

    def run(self, node):
        node.drivetrain_enable.publish(Bool(data=False))
        if node.tof_distance >= 0.06:
            node.raw_cmd.publish(Twist())
            newPose = Pose(position=Point(x=1.14, y=node.position.position.y))
            node.new_pose.publish(newPose)
            node.goal_pub.publish(newPose)
            node.drivetrain_enable.publish(Bool(data=True))
            return self.nextAction

        node.raw_cmd.publish(Twist(linear=Vector3(x=-0.05)))
        
        return self
