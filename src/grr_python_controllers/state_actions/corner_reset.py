#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

from state_actions.action import Action

# This action is meant to take care of placing the April tags in the beginning

class CornerReset(Action):
    def __init__(self, reset_pose:Pose):
        super().__init__()
        
        self.reset_pose = reset_pose
        self.runs = 0

    def run(self, node):
        self.runs += 1
        node.new_pose.publish(self.reset_pose)
        
        if self.runs == 0:
            node.drivetrain_enable.publish(Bool(data=False))
        elif self.runs <= 15:
            print("Resetting")
            node.new_pose.publish(self.reset_pose)
            node.goal_pub.publish(self.reset_pose)
        elif self.runs > 20:
            node.drivetrain_enable.publish(Bool(data=True))
            return self.nextAction
        
        return self
