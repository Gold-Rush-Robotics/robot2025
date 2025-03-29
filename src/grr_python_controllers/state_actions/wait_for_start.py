#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose

from state_actions.action import Action



# This action is meant to take care of placing the April tags in the beginning

class WaitForStart(Action):
    def __init__(self):
        super().__init__()

    def run(self, node):
        node.get_logger().info(f"{node.bools['start_light']} {node.bools['gui_start']}")
        if node.bools["start_light"] and node.bools["gui_start"]:
            return self.nextAction
        return self

        
