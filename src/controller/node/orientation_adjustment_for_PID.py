#!/usr/bin/env python
import rospy
import numpy as np
import math as m

from sensor_processor.msg import Orientation
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

class OrientAdjustForPID():
    def __init__(self, name):
        rospy.init_node(name)

        self.yaw = 0.0
        self.x_setpoint = 0.0
        self.y_setpoint = 0.0

        # PUBLISHER:
        self.thrust_pub = rospy.Publisher("thrust/state", Float64, queue_size=1)
        self.lt_pub = rospy.Publisher("lateral_thrust/state", Float64, queue_size=1)

        # SUBSCRIBER:
        rospy.Subscriber("setpoint/x_inertial", Float64, self.x_setpoint_callback, queue_size=1)
        rospy.Subscriber("setpoint/y_inertial", Float64, self.y_setpoint_callback, queue_size=1)
        rospy.Subscriber("orientation/euler", Orientation, self.orientation_callback, queue_size=1)
        rospy.Subscriber("localization/ekf/baselink_position", Point, self.location_callback, queue_size=1)

    def location_callback(self, msg):
        vec_in_inertial = np.array([self.x_setpoint, self.y_setpoint]) - np.array([msg.x, msg.y])
        # print("vec_in_inertial: " + str(vec_in_inertial))
        rotMatrix = np.matrix([
            [m.cos(self.yaw), m.sin(self.yaw)],
            [-m.sin(self.yaw),  m.cos(self.yaw)],])
        x_dis, y_dis = np.array(np.dot(rotMatrix, vec_in_inertial))[0]
        self.publish_Float64(self.thrust_pub, -x_dis)    # thrust of Rov is y coor
        self.publish_Float64(self.lt_pub, y_dis)        # lateral_thrust of Rov is x coor
        # print("thrust_dis: " + str(round(-x_dis, 4)) + "  lt_dis: " + str(round(y_dis, 4)))

    def orientation_callback(self, msg):
        self.yaw  = msg.yaw
    
    def x_setpoint_callback(self, msg):
        self.x_setpoint = msg.data

    def y_setpoint_callback(self, msg):
        self.y_setpoint = msg.data

    def publish_Float64(self, pub, float64):
        msg = Float64()
        msg.data = float64
        pub.publish(msg)

def main():
    node = OrientAdjustForPID("OrientAdjustForPID")
    rospy.spin()

if __name__ == "__main__":
    main()
