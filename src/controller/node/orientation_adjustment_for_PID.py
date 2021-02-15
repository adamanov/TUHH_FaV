#!/usr/bin/env python
import rospy
import numpy as np
import math as m

from sensor_processor.msg import Orientation
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


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
        # rospy.Subscriber("pathPlanning/setpoints2goal", Point, self.setpoint_callback, queue_size=1)
        rospy.Subscriber("setpoint/x_inertial", Float64, self.setpointX_callback, queue_size=1)
        rospy.Subscriber("setpoint/y_inertial", Float64, self.setpointY_callback, queue_size=1)
        rospy.Subscriber("orientation/euler", Orientation, self.orientation_callback, queue_size=1)
        self.simulation = rospy.get_param("sim")
        if self.simulation:
            rospy.Subscriber("/ground_truth/state", Odometry, self.location_callback, queue_size=1)
        else:
            self.EKF_topic_name = rospy.get_param("EKF_topic_name")
            rospy.Subscriber(self.EKF_topic_name, PoseWithCovarianceStamped, self.location_callback)

    def location_callback(self, msg):
        vec_in_inertial = np.array([self.x_setpoint, self.y_setpoint]) - np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        rotMatrix = np.matrix([
            [m.cos(self.yaw), m.sin(self.yaw)],
            [-m.sin(self.yaw),  m.cos(self.yaw)],])
        x_dis, y_dis = np.array(np.dot(rotMatrix, vec_in_inertial))[0]
        self.publish_Float64(self.thrust_pub, -x_dis)    # thrust of Rov is y coor
        self.publish_Float64(self.lt_pub, y_dis)        # lateral_thrust of Rov is x coor
        # print("dis_X: " + str(x_dis) + "   dis_y: " + str(y_dis))

    def orientation_callback(self, msg):
        self.yaw = msg.yaw
    
    def setpointX_callback(self, msg):
        self.x_setpoint = msg.data

    def setpointY_callback(self, msg):
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
