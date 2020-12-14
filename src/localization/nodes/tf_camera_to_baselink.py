#!/usr/bin/env python
import math as m
import rospy
import numpy as np
from geometry_msgs.msg import Point
from depth_controller.msg import Orientation

class TfCameraToBaselink():
    def __init__(self, name):
        rospy.init_node(name)
        # TODO: Change this parameter before starting the experiment
        rospy.set_param('baselink_to_camera', [0.2, 0, 0.1])

        self.baselink_in_inertial = np.array([[0.0, 0.0, 0.0]]).T
        self.camera_in_inertial = np.array([[0.0, 0.0, 0.0]]).T
        self.baselink_to_camera = np.array([rospy.get_param('baselink_to_camera')]).T
        self.pitch = 0.0
        self.yaw = 1.57
        self.roll = 0.0

        self.pub_baselink_ekf = rospy.Publisher("localization/ekf/baselink_position", Point,queue_size=1)
        self.pub_baselink_least_squares = rospy.Publisher("localization/least_squares/baselink_position", Point,queue_size=1)
        self.pub_baselink_least_squares_and_kf = rospy.Publisher("localization/least_squares_and_kf/baselink_position", Point,queue_size=1)

        rospy.Subscriber("localization/ekf/camera_position", Point, self.ekf_callback)
        rospy.Subscriber("localization/least_squares/camera_position", Point, self.least_squares_callback)
        rospy.Subscriber("localization/least_squares_and_kf/camera_position", Point, self.least_squares_and_kf_callback)

        rospy.Subscriber("orientation/euler", Orientation,
                         self.orientation_callback,
                         queue_size=1)

    def orientation_callback(self, msg):
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = msg.yaw
        print(self.yaw)

    def ekf_callback(self, msg):
        self.calculate_baselink_in_inertial(msg.x, msg.y, msg.z)

        updated_baslink_position = Point()
        updated_baslink_position.x = self.baselink_in_inertial[0][0]
        updated_baslink_position.y = self.baselink_in_inertial[1][0]
        updated_baslink_position.z = self.baselink_in_inertial[2][0]

        self.pub_baselink_ekf.publish(updated_baslink_position)

    def least_squares_callback(self, msg):
        self.calculate_baselink_in_inertial(msg.x, msg.y, msg.z)

        updated_baslink_position = Point()
        updated_baslink_position.x = self.baselink_in_inertial[0][0]
        updated_baslink_position.y = self.baselink_in_inertial[1][0]
        updated_baslink_position.z = self.baselink_in_inertial[2][0]

        self.pub_baselink_least_squares.publish(updated_baslink_position)

    def least_squares_and_kf_callback(self, msg):
        self.calculate_baselink_in_inertial(msg.x, msg.y, msg.z)

        updated_baslink_position = Point()
        updated_baslink_position.x = self.baselink_in_inertial[0][0]
        updated_baslink_position.y = self.baselink_in_inertial[1][0]
        updated_baslink_position.z = self.baselink_in_inertial[2][0]

        self.pub_baselink_least_squares_and_kf.publish(updated_baslink_position)

    def calculate_baselink_in_inertial(self, x, y, z):
        self.camera_in_inertial = np.array([[x, y, z]]).T
        rotation_matrix = self.rotation_matrix_from_euler(self.pitch, self.yaw, self.roll)
        # Rotate baslink to camera vector and substract the resulting vector to get the baslink position in the inertial system
        self.baselink_in_inertial = self.camera_in_inertial - np.dot(rotation_matrix, self.baselink_to_camera)

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

def main():
    node = TfCameraToBaselink("TfCameraToBaselink")
    rospy.spin()

if __name__ == "__main__":
   main()
