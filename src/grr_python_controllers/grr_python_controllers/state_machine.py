#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time

#msg imports
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Twist
from trajectory_msgs.msg import JointTrajectory 
from std_msgs.msg import Bool, Float64, Float64MultiArray
from sensor_msgs.msg import JointState, LaserScan

from collections import defaultdict

from state_actions.wait_for_start import WaitForStart
from state_actions.drive_to_pose import DriveToPose
from state_actions.servo import ServoAction
from state_actions.corner_reset import CornerReset
from state_actions.motor_action import MotorAction
from state_actions.sleep import SleepAction
from state_actions.drive_to_gap import DriveToGap
from state_actions.action import Action



class StateMachine(Node):
    def __init__(self):
        super().__init__('State_Machine')
        
        #self.action_tree = ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechansim_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, 0.0, 100.0]), 1, name="starting positions")
        self.action_tree = ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechansim_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, 50.0, 0.0, 100.0]), 1, name="starting positions")
        self.action_tree.setNext(
            CornerReset(Pose())
        ).setNext(
            WaitForStart()
        ).setNext(    
            DriveToPose(Pose(position=Point(y=0.14)), name="deploy april tag")
        ).setNext(
            ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[97.0, 100.0, 10.0, 0.0, 100.0]), .25, name="Start Sweep")
        ).setNext(
            MotorAction(Float64MultiArray(data=[100.0,-100.0]))
        ).setNext(
            # ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, 0.0, 5.0]), 1, name="Cube Pickup Positions")
            ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[97.0, 100.0, 100.0, 0.0, 5.0]), .5, name="Cube Pickup Positions")
        ).setNext(
            DriveToPose(Pose(position=Point(y=0.01)), name="into cubes")
        ).setNext(
            # ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, 0.0, 0.0]), 1, name="Squeeze")
            ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[97.0, 100.0, -10.0, 0.0, 0.0]), .5, name="Squeeze")
        ).setNext(
            DriveToPose(Pose(position=Point(y=0.03)), name="back up to lift")
        ).setNext(
            ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[97.0, 100.0, -10.0, 0.0, 80.0]), .25, name="Lift")
        ).setNext(
            DriveToPose(Pose(position=Point(y=-0.14)), name="into the wall")
        ).setNext(
            DriveToPose(Pose(position=Point(x=0.05, y=0.12)), name="north wall")
        ).setNext(
            DriveToPose(Pose(position=Point(x=0.15, y=0.12)), name="forward on north wall")
        ).setNext(
            DriveToPose(Pose(position=Point(x=0.15, y=-0.14)), name="to south wall")
        ).setNext(
            DriveToPose(Pose(position=Point(x=0.15)), name="back to center")
        ).setNext(
            MotorAction(Float64MultiArray(data=[100.0, 0.0]))
        ).setNext(
            ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, -10.0, 0.0, 80.0]), 1, name="lift spinny thingy")
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.75)), name="across the hill")
        ).setNext(
            MotorAction(Float64MultiArray(data=[0.0, 0.0]))
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.75, y=-0.2)), name="south wall square")
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.75, y=-0.1)), name="off the wall")
        ).setNext(
            DriveToPose(Pose(position=Point(x=2.35, y=-.2)), name="to the corner")        
        ).setNext(
            CornerReset(Pose(position=Point(x=2.1336, y=-0.145)))
        ).setNext(
            SleepAction(.01)
        ).setNext(
            DriveToPose(Pose(position=Point(x=2.1, y=-0.1)), name="back up a bit before spinning intake")
        ).setNext(
            MotorAction(Float64MultiArray(data=[-100.0, 0.0]))
        ).setNext(
            DriveToPose(Pose(position=Point(x=2.05, y=-0.1)), name="back out of the zone while spinning intake")
        ).setNext(
            SleepAction(1)
        ).setNext(
            MotorAction(Float64MultiArray(data=[0.0, 0.0]))
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.95, y=0.1)), name="out of the corner")
        ).setNext(
            DriveToPose(Pose(position=Point(x=2.1, y=0.1)), name="over a bit")
        ).setNext(
            DriveToPose(Pose(position=Point(x=2.1, y=-0.1)), name="in a bit (push the cubes into the red zone)")
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.95)), name="out of the corner")
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.95), orientation=Quaternion(z=0.707, w=0.707)), name="turn to drop packages")
        ).setNext(
            ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, -10.0, 0.0, 0.0]), .5, name="Place")
        ).setNext(
            ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, 100.0, 0.0, 0.0]), .1, name="Release")
        ).setNext(
            ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, 100.0, 0.0, 80.0]), .25, name="Lift")
        ).setNext(
            ServoAction(JointState(name=['small_package_sweeper_joint', 'bridge_latch_joint', 'mechanism_package_joint', 'mechanism_thruster_joint', 'mechanism_lift_joint'], position=[0.0, 100.0, 10.0, 0.0, 80.0]), .01, name="Close")
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.95, y=0.55), orientation=Quaternion(z=0.707, w=0.707)))
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.95, y=0.55)))
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.24, y=0.55)))
        ).setNext(
            DriveToGap()
        ).setNext(
            DriveToPose(Pose(position=Point(x=1.24, y=0.55)), name="back up for bridge") #CHANGE Y
        ).setNext(
            ServoAction(JointState(name=['bridge_latch_joint'], position=[0.0]), 1, "Drop Bridge")
        ).setNext(
            DriveToPose(Pose(position=Point(x=-0.1,y=0.55)), name="across the bridge into the wall")
        ).setNext(
            CornerReset(Pose(position=Point(x=0.0, y=0.55)))
        ).setNext(
            DriveToPose(Pose(position=Point(x=0.05, y=0.55)), name="back up")
        ).setNext(
            DriveToPose(Pose(position=Point(x=0.05, y=0.55), orientation=Quaternion(z=1.0, w=0.0)), name="Spin")
        ).setNext(
            DriveToPose(Pose(position=Point(x=0.05, y=0.85), orientation=Quaternion(z=1.0, w=0.0)))
        ).setNext(
            Action()
        )
        
        #-.145 m y
        #2.1336m x

        self.state = 0

        #===================================
        # Listeners:
        #===================================
        # Goal Listener
        self.goal_arrival_sub = self.create_subscription(Bool, "/grr/arrived_at_goal", lambda x: self.bool_callback(x, "goal_arrived"), 10)

        self.start_light_sub = self.create_subscription(Bool,'/grr/start_light', lambda x: self.bool_callback(x, "start_light"), 10)
        
        self.gui_start_sub = self.create_subscription(Bool, '/gui/start', lambda x: self.bool_callback(x, "gui_start"), 10)

        self.motor_effort_sub = self.create_subscription(Float64MultiArray, '/effort_controller/commands',self.effort_callback, 10)

        self.tof_sub = self.create_subscription(LaserScan, "/grr/down_tof", self.tof_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        #==========================================
        # Publishers
        #==========================================
        self.goal_pub = self.create_publisher(Pose, "/drivetrain/goal", 10)
        self.servo_pub = self.create_publisher(JointState, "/grr/joint_command", 10)
        self.drivetrain_enable = self.create_publisher(Bool, "/drivetrain/publish", 10)
        self.new_pose = self.create_publisher(Pose, "/pose", 10)
        self.effort_pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.raw_cmd = self.create_publisher(Twist, "/grr_cmake_controller/cmd_vel_unstamped", 10)      

        self.tof_distance = 0.0
        self.position = Pose()

        # I define a goal that can be modified throughout the program and sent when new goals are needed
        self.goal = Pose()

        self.current_motor_effort = Float64MultiArray()
        
        self.bools = defaultdict(lambda: False)
        
        self.runner_timer = self.create_timer(1/30.0, self.runner)
        
        self.current_action_name = ""
        
    def tof_callback(self, msg:LaserScan):
        self.tof_distance = msg.ranges[0]
        
    def odom_callback(self, msg:Odometry):
        self.position = msg.pose.pose
        
    def runner(self):
        self.action_tree = self.action_tree.run(self)
        if self.action_tree.name != self.current_action_name:
            self.get_logger().info(f"Starting: {self.action_tree.name}")
            self.current_action_name = self.action_tree.name
        
    def bool_callback(self, msg:Bool, name:str):
        self.bools[name] = msg.data
    
    def effort_callback(self, msg:Float64MultiArray):
        self.motor_effort = msg


def main():
    rclpy.init()
    Renwick = StateMachine()
    rclpy.spin(Renwick)



if __name__ == '__main__':
    main()
