#!/usr/bin/env python
import rospy
import numpy as np
import time
import math as m
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64


class setpointsGenerator:
    def __init__(self, name):
        rospy.init_node(name)
        # Set a desired position of robot
        # self.pose = Point()
        # self.angels = Point()

        self.setpointPoseX = 1
        self.setpointPoseY = 1
        self.setpointPoseZ = -0.4
        # Set a desired orientation of robot
        self.setpointAngleRoll = 0
        self.setpointAnglePitch = 0
        self.setpointAngleYaw = 0

        # Set as a parameter for each
        rospy.set_param('setpointPoseX', 1)
        rospy.set_param('setpointPoseY', 2.0)
        rospy.set_param('setpointPoseZ', -0.4)

        # rospy.set_param('setpointAngleRoll', 0)
        # rospy.set_param('setpointAnglePitch', 0)
        rospy.set_param('setpointAngleYaw', 90*2*m.pi/360)

        # Bounds given by tank dimensions
        rospy.set_param('safezone_upper', -0.2)  # , -0.15)
        rospy.set_param('safezone_lower', -0.7)  # , -0.6)
        rospy.set_param('safezone_left_x', 0.49)
        rospy.set_param('safezone_right_x', 1.21)
        rospy.set_param('safezone_front_y', 2.5)  # direction of where tags are
        rospy.set_param('safezone_back_y', 1.0)

        # PUBLISHER:
        self.thrust_setpoint_pub = rospy.Publisher("thrust/setpoint", Float64, queue_size=1)
        self.vt_setpoint_pub = rospy.Publisher("vertical_thrust/setpoint", Float64, queue_size=1)
        self.lt_setpoint_pub = rospy.Publisher("lateral_thrust/setpoint", Float64, queue_size=1)
        self.yaw_setpoint_pub = rospy.Publisher("yaw/setpoint", Float64, queue_size=1)
        # self.pitch_setpoint_pub = rospy.Publisher("pitch/setpoint", Float64, queue_size=1)
        # self.roll_setpoint_pub = rospy.Publisher("roll/setpoint", Float64, queue_size=1)

    def calculate_setpoint(self):
        if (rospy.get_param("setpointPoseX") < rospy.get_param("safezone_right_x")) and \
         (rospy.get_param("setpointPoseX") > rospy.get_param("safezone_left_x")):
            self.setpointPoseX = rospy.get_param("setpointPoseX")
        else:
            rospy.loginfo("Please set setpointPoseX to float in safezone!")

        if (rospy.get_param("setpointPoseY") < rospy.get_param("safezone_front_y")) and \
         (rospy.get_param("setpointPoseY") > rospy.get_param("safezone_back_y")):
            self.setpointPoseY = rospy.get_param("setpointPoseY")
        else:
            rospy.loginfo("Please set setpointPoseY to float in safezone!")
        
        if (rospy.get_param("setpointPoseZ") < rospy.get_param("safezone_upper")) and \
         (rospy.get_param("setpointPoseZ") > rospy.get_param("safezone_lower")):
            self.setpointPoseZ = rospy.get_param("setpointPoseZ")
        else:
            rospy.loginfo("Please set setpointPoseZ to float in safezone!")



    def publish_Float64(self, pub, float):
        msg = Float64()
        msg.data = float
        pub.publish(msg)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            # Publish a desired position of robot
            self.calculate_setpoint()   # check if the depth setpoint is valid
            self.x = self.setpointPoseX
            self.y = self.setpointPoseY
            self.z = self.setpointPoseZ
            self.publish_Float64(self.lt_setpoint_pub, self.x )
            self.publish_Float64(self.thrust_setpoint_pub, self.y)
            self.publish_Float64(self.vt_setpoint_pub, self.z)

            # Publish a desired orientation of robot
            # self.angels.x = rospy.get_param('setpointAngleRoll')
            # self.angels.y = rospy.get_param('setpointAnglePitch')
            self.z = rospy.get_param('setpointAngleYaw')
            self.publish_Float64(self.yaw_setpoint_pub, self.z)
            rate.sleep()


if __name__ == "__main__":
    node = setpointsGenerator("multiSetpointGenerator")
    node.run()
