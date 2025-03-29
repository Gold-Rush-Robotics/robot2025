#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import JointState

from state_actions.action import Action

# This action is meant to take care of placing the April tags in the beginning

class ServoAction(Action):
    def __init__(self, servo_command:JointState, timer_sec:float, name=None):
        super().__init__()
        self.command = servo_command
        self.timer_sec = timer_sec
        self.timer_done = False
        self.timer = None
        if name:
            self.name = name
    
    def timer_callback(self):
        self.timer_done = True

    def run(self, node:Node):
        if self.timer_done:
            self.timer.destroy()
            return self.nextAction
        
        if not self.timer:
            self.timer = node.create_timer(self.timer_sec, self.timer_callback)
        
        node.servo_pub.publish(self.command)
        
        return self
