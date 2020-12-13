#!/usr/bin/env python
import rospy
import numpy as np
from range_sensor.msg import RangeMeasurementArray
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance 
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

class LocalizationError():
    def __init__(self, name):
        rospy.init_node(name)

        self.ground_truth_location = Point()
        self.estimated_global_position = Point()

        rospy.Subscriber("/ground_truth/state", Odometry, self.ground_truth_callback)

        rospy.Subscriber("localization/ekf/baselink_position", Point, self.global_position_callback)

        self.pub_error_dis = rospy.Publisher("localization/error/distance", Float64, queue_size=1)

        self.pub_error_vec = rospy.Publisher("localization/error/vector", Point, queue_size=1)

    def ground_truth_callback(self, msg):
        #self.ground_truth_location = msg.PoseWithCovariance.pose.Point
        self.ground_truth_location = msg.pose.pose.position
        # print("Actual pos: x " + str(self.ground_truth_location.x) + " y: " + str(self.ground_truth_location.y) + " z: " + str(self.ground_truth_location.z))
    
    def global_position_callback(self, msg):
        self.estimated_global_position = msg
        msg1 = Float64()
        msg1.data = round(self.distance2points_scalar(self.ground_truth_location, self.estimated_global_position), 3)
        self.pub_error_dis.publish(msg1)
        msg2 = self.distance2points_vec(self.ground_truth_location, self.estimated_global_position)
        self.pub_error_vec.publish(msg2)

    def distance2points_scalar(self, P1, P2):
        return np.sqrt((P1.x - P2.x)**2 + (P1.y - P2.y)**2 + (P1.z - P2.z)**2)

    def distance2points_vec(self, P1, P2):
        vec = Point()
        vec.x, vec.y, vec.z = round((P1.x - P2.x), 3), round((P1.y - P2.y),3), round((P1.z - P2.z), 3)
        return vec

def main():
    node = LocalizationError("LocalizationError")
    rospy.spin()

if __name__ == "__main__":
   main()
