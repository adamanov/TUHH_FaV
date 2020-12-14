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

        #SUBSCRIBER:
        rospy.Subscriber("/ground_truth/state", Odometry, self.ground_truth_callback)
        rospy.Subscriber("localization/ekf/baselink_position", Point, self.ekf_callback)
        rospy.Subscriber("localization/least_squares/baselink_position", Point, self.ls_callback)
        rospy.Subscriber("localization/least_squares_and_kf/baselink_position", Point, self.lskf_callback)
        
        # PUBLISHER:
        self.pub_ekf_error_dis = rospy.Publisher("localization/ekf/error/distance", Float64, queue_size=1)
        self.pub_ekf_error_vec = rospy.Publisher("localization/ekf/error/vector", Point, queue_size=1)
        self.pub_ls_error_dis = rospy.Publisher("localization/least_squares/error/distance", Float64, queue_size=1)
        self.pub_ls_error_vec = rospy.Publisher("localization/least_squares/error/vector", Point, queue_size=1)
        self.pub_lskf_error_dis = rospy.Publisher("localization/least_squares_and_kf/error/distance", Float64, queue_size=1)
        self.pub_lskf_error_vec = rospy.Publisher("localization/least_squares_and_kf/error/vector", Point, queue_size=1)

    def ground_truth_callback(self, msg):
        self.ground_truth_location = msg.pose.pose.position
    
    def ekf_callback(self, msg):
        self.universal_callback(msg, self.pub_ekf_error_vec, self.pub_ekf_error_dis)

    def ls_callback(self, msg):
        self.universal_callback(msg, self.pub_ls_error_vec, self.pub_ls_error_dis)
    
    def lskf_callback(self, msg):
        self.universal_callback(msg, self.pub_lskf_error_vec, self.pub_lskf_error_dis)

    def universal_callback(self, msg, pub_vec, pub_dis):
        estimated_position = msg
        msg1 = Float64()
        msg1.data = round(self.distance2points_scalar(self.ground_truth_location, estimated_position), 3)
        pub_dis.publish(msg1)
        msg2 = self.distance2points_vec(self.ground_truth_location, estimated_position)
        pub_vec.publish(msg2)

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
