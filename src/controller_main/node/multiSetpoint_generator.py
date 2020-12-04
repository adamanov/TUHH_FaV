#!/usr/bin/env python
import rospy
import numpy as np
import time
import math as m
from nav_msgs.msg import Odometry
from geometry_msgs import Point


class setpointsGenerator:
    def __init__(self, name):
        rospy.init_node(name)

        # Set a desired position of robot
        self.setpointPoseX = 1
        self.setpointPoseY = 1
        self.setpointPoseZ = -0.7

        # Set a desired orientation of robot
        self.setpointRoll = 0
        self.setpointPitch = 0
        self.setpointYaw = 90

        # Set as a parameter for each
        rospy.set_param('setpointPoseX', 1)
        rospy.set_param('setpointPoseY', 1)
        rospy.set_param('setpointPoseZ', -0.5)

        # Option to get manually parameters
        rospy.get_param('setpointPoseX')
        rospy.get_param('setpointPoseY')

        # Upper and lower bound of depth
        rospy.set_param('safezone_upper', -0.15)
        rospy.set_param('safezone_lower', -0.6)

        # Subscribe to a topic to publish a position of robot
        self.setpointsPose_pub = rospy.Publisher("desired_pose/setpoint",
                                                 Point,
                                                 queue_size=1)

        # Subscribe to a topic to publish a orientation of robot
        self.setpointOrientation_pub = rospy.Publisher(
            "desired_angle/setpoint", Point, queue_size=1)

    def calculate_setpoint(self):
        if self.isValidSetpoint():
            self.setpointPoseZ = rospy.get_param("setpointPoseZ")
        else:
            rospy.loginfo("Please set setpointPoseZ to float in safezone!")

    # Check if given depth setpoint is valid
    def isValidSetpoint(self):
        return (rospy.get_param("setpointPoseZ") < rospy.get_param("safezone_upper")) and (rospy.get_param("setpointPoseZ") > rospy.get_param("safezone_lower"))

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():

            # Publish a desired position of robot
            pose = Point()
            self.calculate_setpoint()   # check if the depth setpoint is valid
            pose.x = self.setpointPoseX
            pose.y = self.setpointPoseY
            pose.z = self.setpointPoseZ
            self.setpointsPose_pub.publish(pose)

            # Publish a desired orientation of robot
            angels = Point()
            angles.x = self.setpointRoll
            angles.y = self.setpointPitch
            angels.z = self.setpointYaw
            self.setpointOrientation_pub.publish(angels)

            rate.sleep()


if __name__ == "__main__":
    node = setpointsGenerator("multiSetpointGenerator")
    node.run()
