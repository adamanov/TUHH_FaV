#!/usr/bin/env python
import rospy
import numpy as np
import time
import math as m
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_processor.msg import Orientation


class ControllerNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.path_following_active = False

        self.hole_hight = 0.5
        self.depth_threshold = 0.5

        self.yaw_current = 0.0

        self.setpoint_distance = 0.1
        
        self.AngleYawDifference = 0.0

        # yaw rotation
        self.yaw_adjust_interval = 0.05 # [s]
        self.last_yaw_adjust = 0

        # Destination Points
        self.destinationX = [75, 60, 85]
        self.destinationY = [250, 50, 300]
        self.destinationPointNr = 0

        # Desired position of robot
        self.setpointPoseX = 0
        self.setpointPoseY = 0
        self.setpointPoseZ = -0.3
        # Set a desired orientation of robot
        self.setpointAngleRoll = 0
        self.setpointAnglePitch = 0
        self.setpointAngleYaw = 0

        # PUBLISHER:
        self.thrust_setpoint_pub = rospy.Publisher("thrust/setpoint", Float64, queue_size=1)
        self.x_setpoint_pub = rospy.Publisher("setpoint/x_inertial", Float64, queue_size=1)
        self.y_setpoint_pub = rospy.Publisher("setpoint/y_inertial", Float64, queue_size=1)
        self.vt_setpoint_pub = rospy.Publisher("vertical_thrust/setpoint", Float64, queue_size=1)
        self.lt_setpoint_pub = rospy.Publisher("lateral_thrust/setpoint", Float64, queue_size=1)
        self.yaw_setpoint_pub = rospy.Publisher("yaw/setpoint", Float64, queue_size=1)
        # self.pitch_setpoint_pub = rospy.Publisher("pitch/setpoint", Float64, queue_size=1)
        # self.roll_setpoint_pub = rospy.Publisher("roll/setpoint", Float64, queue_size=1)

        # SUBSCRIBER:
        rospy.Subscriber("pathPlanning/setpoints2goal", Point, self.setpoint_callback_xy, queue_size=1)
        rospy.Subscriber("wall/pose", Pose, self.wall_callback)
        rospy.Subscriber("/ground_truth/state", Odometry, self.ground_truth_callback, queue_size=1)
        rospy.Subscriber("orientation/euler", Orientation, self.euler_callback, queue_size=1)
    
    def euler_callback(self, msg):
        self.yaw_current = msg.yaw

    def wall_callback(self, msg):
        self.setpointPoseZ = msg.position.z - 0.5*self.hole_hight
        print("setpoint Z: " + str(self.setpointPoseZ))
        if abs(self.setpointPoseZ - self.currentPosition[2]) <= self.depth_threshold:
            if self.path_following_active == False:
                print("Turned ON path following (in depth range)")
            self.path_following_active = True
        else:
            if self.path_following_active == True:
                print("Turned OFF path following (out off depth range)")
            self.path_following_active = False


    def setpoint_callback_xy(self, msg):
        self.setpointPoseX = msg.x
        self.setpointPoseY = msg.y
    
    def ground_truth_callback(self, msg):
        self.currentPosition = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        if self.setpoint_distance >= np.linalg.norm(np.array(self.currentPosition[0:2]) - \
        np.array([self.destinationX[self.destinationPointNr], self.destinationY[self.destinationPointNr]])) \
        and self.path_following_active == True:
            self.path_following_active = False
            self.destinationPointNr = max(len(self.destinationX) - 1, self.destinationPointNr + 1)
            rospy.set_param('destinationX', self.destinationX[self.destinationPointNr])
            rospy.set_param('destinationY', self.destinationY[self.destinationPointNr])
            print("Turned OFF path following (at destination point)")

    def calculate_yaw(self):
        if (rospy.get_time() - self.last_yaw_adjust > self.yaw_adjust_interval):
            self.setpointAngleYaw = (self.setpointAngleYaw + 1*m.pi*2/360) % (2*m.pi) # between zero an 2*pi
            difference = (self.yaw_current + m.pi) - self.setpointAngleYaw
            if m.pi >= abs(difference):
                self.AngleYawDifference = difference
            elif difference >= 0:
                self.AngleYawDifference = difference - 2*m.pi
            elif difference < 0:
                self.AngleYawDifference = difference + 2*m.pi
            print("AngleYawDiffrence: " + str(self.AngleYawDifference))
            self.last_yaw_adjust = rospy.get_time()
            
            

    def publish_Float64(self, pub, float):
        msg = Float64()
        msg.data = float
        pub.publish(msg)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.calculate_yaw()
            if self.path_following_active:
                # Publish states and setpoints x,y to PID Controller
                self.publish_Float64(self.x_setpoint_pub, self.setpointPoseX)
                self.publish_Float64(self.y_setpoint_pub, self.setpointPoseY)
                self.publish_Float64(self.lt_setpoint_pub, 0.0 )
                self.publish_Float64(self.thrust_setpoint_pub, 0.0)
                # print("Path Following is running")
            # publish setpoint Z
            self.publish_Float64(self.vt_setpoint_pub, self.setpointPoseZ)
            # publish desired Yaw ANgle
            self.publish_Float64(self.yaw_setpoint_pub, -self.AngleYawDifference)
            rate.sleep()


if __name__ == "__main__":
    node = ControllerNode("ControllerNode")
    node.run()
