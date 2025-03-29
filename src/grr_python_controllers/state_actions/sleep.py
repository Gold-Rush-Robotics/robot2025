#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import JointState

from state_actions.action import Action

# This action is meant to take care of placing the April tags in the beginning

class SleepAction(Action):
    def __init__(self, sleep_sec:float):
        super().__init__()
        self.sleep_sec = sleep_sec

    def run(self, node:Node):
        time.sleep(self.sleep_sec)
        
        return self.nextAction
