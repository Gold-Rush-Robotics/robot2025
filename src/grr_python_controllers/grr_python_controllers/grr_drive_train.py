import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class PIDcontroller:

    #Constructor
    def __init__(self, Kp, Ki, Kd, dt) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.dt = dt

    #update the control singal
    def update(self, error) -> float:
        
        #Porportional response
        P = self.Kp * error
        
        #Integral response
        self.integral += error * self.dt
        I = self.Ki * self.integral

        #Derivative response
        D = self.Kd * ((error - self.prev_error)/ self.dt)

        #Output
        control_signal = P + I + D

        #Update prev_error
        self.prev_error = error

        #return
        return control_signal

class DriveTrain(Node):
    
    #Constructor
    def __init__(self, xPID, yPID, rPID, dt) -> None:
        super().__init__('DriveTrain')

        # Define publishers
        self.velocity_publisher = self.create_publisher(Twist, "/grr_cmake_controller/cmd_vel_unstamped", 10)
        # TODO: Does a publisher have to have a hz rate? Like can I just publish when I want? Will publish true for this constantly mess up future trips?
        self.arrival_publisher = self.create_publisher(Bool, "arrived_at_goal", 10)

        # Define subscribers
        self.odom_subscriber = self.create_subscription(Odometry, "/odom", self.getOdom, 10)
        self.goal_subscriber = self.create_subscription(Pose, "/drivetrain/goal", self.getGoal, 10)
        
        self.publish_subscriber = self.create_subscription(Bool, "/drivetrain/publish", self.publish_subscribe_callback, 10)

        # Define Current pose parameters
        self.currentPose = Pose()
        self.currentPose.position.x = 0.0
        self.currentPose.position.y = 0.0
        self.currentPose.orientation.w = 0.0
        self.currentPose.orientation.x = 0.0
        self.currentPose.orientation.y = 0.0
        self.currentPose.orientation.z = 0.0

        # Define goal parameter
        self.goal = Pose()
        self.goal.position.x = 0.0
        self.goal.position.y = 0.0
        self.currentPose.orientation.w = 0.0
        self.currentPose.orientation.x = 0.0
        self.currentPose.orientation.y = 0.0
        self.currentPose.orientation.z = 0.0

        # Initialize PID controller
        self.translateXPID = PIDcontroller(xPID[0], xPID[1], xPID[2], (dt * 0.01))
        self.translateYPID = PIDcontroller(yPID[0], yPID[1], yPID[2], (dt * 0.01))
        self.rotatePID = PIDcontroller(rPID[0], rPID[1], rPID[2], (dt * 0.01))
        
        self.translate_tolerance = .01
        self.rotate_tolerance = .05
        
        self.publish = True
        
    def publish_subscribe_callback(self, msg:Bool):
        self.publish = msg.data


    # gets the current odometry of the robot
    def getOdom(self, odom: Odometry):
        odom = odom.pose.pose
        self.currentPose.position.x = odom.position.x
        self.currentPose.position.y = odom.position.y
        self.currentPose.orientation.w = odom.orientation.w
        self.currentPose.orientation.z = odom.orientation.z
        self.setVelocity()


    # Update goal of the drive train
    def getGoal(self, goal: Pose) -> None:
        if self.goal.position.x != goal.position.x or self.goal.position.y != goal.position.y or self.goal.orientation.z != goal.orientation.z:

            print("goal Recieved")
            
            arrival = Bool()
            arrival.data = False
            self.arrival_publisher.publish(arrival)

            # Get the linear goals
            self.goal.position.x = goal.position.x
            self.goal.position.y = goal.position.y

            # Get the rotational goals
            self.goal.orientation.w = goal.orientation.w
            self.goal.orientation.z = goal.orientation.z

    # sets the current planned x and y movement
    def setVelocity(self):
        if not self.publish:
            return

        min_vel = 0.05
        min_rot = 0.1

        # Define the error (difference between where you are and where you want to be)
        x_error = self.goal.position.x - self.currentPose.position.x
        y_error = self.goal.position.y - self.currentPose.position.y
        theta_error = self.normalize_angle(self.quatToEuler(self.goal) - self.quatToEuler(self.currentPose))

        # Build the message
        velocity = Twist()
        
        within_x = abs(x_error) <= self.translate_tolerance
        within_y = abs(y_error) <= self.translate_tolerance
        within_theta = abs(theta_error) <= self.rotate_tolerance
        
        if not within_x:
            velocity.linear.x = self.minimumfunction(self.translateXPID.update(x_error), min_vel)
        
        if not within_y:
            velocity.linear.y = self.minimumfunction(self.translateYPID.update(y_error), min_vel)
        
        if not within_theta:
            velocity.angular.z = self.minimumfunction(self.rotatePID.update(theta_error), min_rot)

        print(velocity)

        # Publish
        self.velocity_publisher.publish(velocity)
        within = within_x and within_y and within_theta
        self.arrival_publisher.publish(Bool(data=bool(within)))

        # Check to see if we have arrived
    # Converts Quaternions to Euler angles
    # So we don't really care about most of the Quaternion because we are only going to rotate about the Z axis
    def quatToEuler(self, pose: Pose):
        QW = pose.orientation.w
        QZ = pose.orientation.z

        SyCp = 2 * (QW * QZ)
        CyCp = 1 - (2 * (QZ * QZ))

        angles = np.arctan2(SyCp, CyCp)


        return angles
    
    # Takes in a value and takes in a minimum 
    def minimumfunction(self, val, min):

        valsign = np.sign(val)

        val = abs(val)

        val = max(val, min)

        return valsign * val
    
    def normalize_angle(self, angle:float) -> float:
        return np.arctan2(np.sin(angle), np.cos(angle))


def main():
    
    #Define PID values
    xPID = [.5, 0, 0]
    yPID = [.4, 0, 0]
    rotatePID = [1.145, 0, 0]
    
    rclpy.init()

    driveTrain = DriveTrain(xPID, yPID, rotatePID, 10)
    rclpy.spin(driveTrain)

if __name__ == '__main__':
    main()


