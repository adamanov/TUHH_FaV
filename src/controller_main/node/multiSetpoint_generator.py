#!/usr/bin/env python
import rospy
import numpy as np
import time
import math as m
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class setpointsGenerator:
    def __init__(self, name):
        rospy.init_node(name)
        # Set a desired position of robot
        self.setpointPoseX = 1
        self.setpointPoseY = 1
        self.setpointPoseZ = -0.5
        # Set a desired orientation of robot
        self.setpointAngleRoll = 0
        self.setpointAnglePitch = 0
        self.setpointAngleYaw = 90
        # Set as a parameter for each
        rospy.set_param('setpointPoseX', 1)
        rospy.set_param('setpointPoseY', 1)
        rospy.set_param('setpointPoseZ', -0.5)
        rospy.set_param('setpointAngleRoll', 0)
        rospy.set_param('setpointAnglePitch', 0)
        rospy.set_param('setpointAngleYaw', 90)

        # Option to get manually parameters
        # rospy.get_param('setpointPoseX')
        # rospy.get_param('setpointPoseY')
        # Get Angles
        # rospy.get_param('setpointAngleRoll')
        # rospy.get_param('setpointAnglePitch')
        # rospy.get_param('setpointAngleYaw')
        # Bounds of depth
        rospy.set_param('safezone_upper', -0.15)
        rospy.set_param('safezone_lower', -0.6)
        rospy.set_param('safezone_left_x', 0.2)
        rospy.set_param('safezone_right_x', 1.3)
        rospy.set_param('safezone_front_y', 1.3)  # direction of where tags are
        rospy.set_param('safezone_back_y', 0.2)
        # Subscribe to a topic to publish a position of robot
        self.setpointsPose_pub = rospy.Publisher("desired_pose/setpoint",
                                                 Point,
                                                 queue_size=1)
        # Subscribe to a topic to publish a orientation of robot
        self.setpointOrientation_pub = rospy.Publisher(
            "desired_angle/setpoint", Point, queue_size=1)

    def calculate_setpoint(self):
        if self.isValidSetpoint():
            self.setpointPoseX = rospy.get_param("setpointPoseX")
            self.setpointPoseY = rospy.get_param("setpointPoseY")
            self.setpointPoseZ = rospy.get_param("setpointPoseZ")
        else:
            rospy.loginfo(
                "Please set setpointPoseX or setpointPoseY or setpointPoseZ to float in safezone!")

    # Check if given depth setpoint is valid
    def isValidSetpoint(self):
        return (rospy.get_param("setpointPoseZ") < rospy.get_param("safezone_upper")) and (rospy.get_param("setpointPoseZ") > rospy.get_param("safezone_lower"))

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():

            # Publish a desired position of robot
            pose = Point()
            # self.calculate_setpoint()   # check if the depth setpoint is valid
            self.pose.x = self.setpointPoseX
            self.pose.y = self.setpointPoseY
            self.pose.z = self.setpointPoseZ
            # print("Setpoint Pose", "\n", self.pose)   # worked
            self.setpointsPose_pub.publish(pose)

            # Publish a desired orientation of robot
            self.angels = Point()
            self.angels.x = self.setpointAngleRoll
            self.angels.y = self.setpointAnglePitch
            self.angels.z = self.setpointAngleYaw
            # print("Setpoint Angeles", "\n", self.angels)  # worked
            self.setpointOrientation_pub.publish(angels)

            rate.sleep()


if __name__ == "__main__":
    node = setpointsGenerator("multiSetpointGenerator")
    node.run()
