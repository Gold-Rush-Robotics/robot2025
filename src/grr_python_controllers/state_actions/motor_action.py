#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose

from std_msgs.msg import Float64, Float64MultiArray

from state_actions.action import Action

# This action is meant to take care of placing the April tags in the beginning

class MotorAction(Action):
    def __init__(self, effort:Float64MultiArray):
        super().__init__()
        self.motor_effort = effort
        

    def run(self, node):
        msg = Float64MultiArray()
        node.effort_pub.publish(self.motor_effort)
        return self.nextAction
