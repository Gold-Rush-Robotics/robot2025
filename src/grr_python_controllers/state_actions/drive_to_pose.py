#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose

from state_actions.action import Action

# This action is meant to take care of placing the April tags in the beginning

class DriveToPose(Action):
    def __init__(self, pose:Pose, name:str=None):
        super().__init__()
        
        self.goal_pose = pose
        
        self.runs = 0
        
        if name:
            self.name = name

    def run(self, node):
        node.goal_pub.publish(self.goal_pose)
        self.runs += 1
        if node.bools["goal_arrived"] and self.runs >= 5:
            return self.nextAction
        return self
