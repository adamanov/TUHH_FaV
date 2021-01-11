#!/usr/bin/env python
import numpy as np
import math
from geometry_msgs.msg import Point
import rospy
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement
from scipy.optimize import lsq_linear

class ObjectDetection():
    def __init__(self,name):

        self.box_position_msg = Point()
        self.tag_positions = [np.array([[0.3,0,-0.2]]).T, np.array([[-0.3,0,-0.2]]).T, np.array([[0.3,0,0.2]]).T, np.array([[-0.3,0,0.2]]).T]
        self.robot_position = np.zeros((3,1))
        self.box_position = np.zeros((3,1))
        rospy.init_node(name)
        rospy.set_param('prediction_rate',20.0)
        self.prediction_rate = rospy.get_param('prediction_rate')        
        self.coord_pub = rospy.Publisher("objectDetection/coordinate", Point, queue_size=1)
        # TODO: Where do we get the ranges for object detection?
        rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback)
        rospy.Subscriber("localization/ekf/camera_position", Point, self.position_callback)   
        
    def range_callback(self,msg):
        self.ranges = dict()
        for i in msg.measurements:
            self.ranges[i.id-1] = i.range
        # print(self.ranges)
        if len(self.ranges) == 4:
            self.calculate_box_position(self.ranges, self.tag_positions, self.robot_position)
            self.get_error()

    def position_callback(self,msg):
        self.robot_position[0] = msg.x
        self.robot_position[1] = msg.y
        self.robot_position[2] = msg.z
    
    def get_error(self):
        ground_truth = np.array([[0.5, 3.35, -0.5]]).T + self.tag_positions[0]
        error_norm = np.linalg.norm(ground_truth-self.box_position)
        print(error_norm)

    def publish(self):
        self.coord_pub.publish(self.box_position_msg)  
        
    def run(self):
        rate = rospy.Rate(self.prediction_rate)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def calculate_box_position(self, ranges, tag_positions, robot_positon):
        A = np.zeros((4, 3))
        b = np.zeros((4, 1))
        # TODO: How should we choose the intial value?
        initial_guess = np.zeros((3, 1))
        for i in range(0,4):
            diff = initial_guess - (robot_positon + tag_positions[i])
            # print(diff)
            grad = diff/np.linalg.norm(diff)
            A[i,:] = grad.T
            b[i] = ranges[i] - np.linalg.norm(diff) + np.dot(grad.T, initial_guess)
            # print("b[i]:"+str(b[i]))
        self.box_position = lsq_linear(A, b.ravel()).x
        # print(self.box_position)
        self.box_position_msg.x = self.box_position[0]
        self.box_position_msg.y = self.box_position[1]
        self.box_position_msg.z = self.box_position[2]


def main():
    node = ObjectDetection("ObjectDetection")
    node.run()
    
if __name__ == "__main__":
    main()