#!/usr/bin/env python
import math as m
import rospy
import numpy as np
from geometry_msgs.msg import Point
from sensor_processor.msg import Orientation
from std_msgs.msg import Float64

class TfCameraToBaselink():
    def __init__(self, name):
        rospy.init_node(name)
        
        # TODO: Change this parameter before starting the experiment
        rospy.set_param('baselink_to_camera', [0.2, 0, 0.1])

        # self.baselink_in_inertial = np.array([[0.0, 0.0, 0.0]]).T
        # self.camera_in_inertial = np.array([[0.0, 0.0, 0.0]]).T
        self.baselink_to_camera = np.array([rospy.get_param('baselink_to_camera')]).T
        self.pitch = 0.0
        self.yaw = 1.57 # (90 Grad)
        self.roll = 0.0

        # PUBLISHER:
        # for Error an Plotting
        self.pub_baselink_ekf = rospy.Publisher("localization/ekf/baselink_position", Point,queue_size=1)
        self.pub_baselink_least_squares = rospy.Publisher("localization/least_squares/baselink_position", Point,queue_size=1)
        self.pub_baselink_least_squares_and_kf = rospy.Publisher("localization/least_squares_and_kf/baselink_position", Point,queue_size=1)
        #for PID
        self.thrust_pub = rospy.Publisher("thrust/state", Float64, queue_size=1)
        self.lt_pub = rospy.Publisher("lateral_thrust/state", Float64, queue_size=1)
        self.vt_pub = rospy.Publisher("vertical_thrust/state", Float64, queue_size=1)
        
        # SUBSCRIBER:
        rospy.Subscriber("localization/ekf/camera_position", Point, self.ekf_callback)
        rospy.Subscriber("localization/least_squares/camera_position", Point, self.least_squares_callback)
        rospy.Subscriber("localization/least_squares_and_kf/camera_position", Point, self.least_squares_and_kf_callback)

        rospy.Subscriber("orientation/euler", Orientation, self.orientation_callback, queue_size=1)

    def orientation_callback(self, msg):
        self.roll, self.pitch, self.yaw  = msg.roll, msg.pitch, msg.yaw

    def ekf_callback(self, msg):
        msg_pub = self.calculate_baselink_in_inertial(msg.x, msg.y, msg.z)
        self.pub_baselink_ekf.publish(msg_pub)
        self.publish_Float64(self.thrust_pub, msg_pub.y) # thrust of Rov is y coor
        self.publish_Float64(self.lt_pub, msg_pub.x)      # lateral_thrust of Rov is x coor
        self.publish_Float64(self.vt_pub, msg_pub.z)

    def least_squares_callback(self, msg):
        msg_pub = self.calculate_baselink_in_inertial(msg.x, msg.y, msg.z)
        self.pub_baselink_least_squares.publish(msg_pub)

    def least_squares_and_kf_callback(self, msg):
        msg_pub = self.calculate_baselink_in_inertial(msg.x, msg.y, msg.z)
        self.pub_baselink_least_squares_and_kf.publish(msg_pub)

    def calculate_baselink_in_inertial(self, x, y, z):
        camera_in_inertial = np.array([[x, y, z]]).T
        rotation_matrix = self.rotation_matrix_from_euler(self.pitch, self.yaw, self.roll)
        # Rotate baslink to camera vector and substract the resulting vector to get the baslink position in the inertial system
        baselink_in_inertial = camera_in_inertial - np.dot(rotation_matrix, self.baselink_to_camera)
        msg = Point()
        msg.x, msg.y, msg.z = baselink_in_inertial[0][0], baselink_in_inertial[1][0], baselink_in_inertial[2][0]
        return msg

    def rotation_matrix_from_euler(self, pitch, yaw, roll):

        yawMatrix = np.matrix([
            [m.cos(yaw), -m.sin(yaw), 0],
            [m.sin(yaw),  m.cos(yaw), 0],
            [0,           0,          1]
            ])

        pitchMatrix = np.matrix([
            [m.cos(pitch),  0, m.sin(pitch) ],
            [0,             1, 0            ],
            [-m.sin(pitch), 0, m.cos(pitch) ]
            ])

        rollMatrix = np.matrix([
            [1, 0,            0          ],
            [0, m.cos(roll), -m.sin(roll)],
            [0, m.sin(roll),  m.cos(roll)]
            ])
        return np.dot(np.dot(yawMatrix, pitchMatrix), rollMatrix)

    def publish_Float64(self, pub, float64):
        msg = Float64()
        msg.data = float64
        pub.publish(msg)

def main():
    node = TfCameraToBaselink("TfCameraToBaselink")
    rospy.spin()

if __name__ == "__main__":
   main()
