#!/usr/bin/env python
import rospy
import numpy as np
import time
import math as m

from geometry_msgs.msg import Point
from path_planning.msg import PathCoordinates
from nav_msgs.msg import Odometry

class SetpointPublisher:
    def __init__(self, name):
        rospy.init_node(name)

        self.setpointVectorX = []
        self.setpointVectorY = []

        self.current_position = [0, 0]

        self.setpoint_distance = 15

        # PUBLISHER:
        self.pub_setPoint= rospy.Publisher("pathPlanning/setpoints2goal", Point, queue_size=1)

        # SUBSCRIBER:
        rospy.Subscriber("path/coordinates", PathCoordinates, self.path_callback, queue_size=1)
        rospy.Subscriber("/ground_truth/state", Odometry, self.ground_truth_callback, queue_size=1)

    def ground_truth_callback(self, msg):
        self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])*100

    def path_callback(self, msg):
        self.setpointVectorX = msg.x_array
        self.setpointVectorY = msg.y_array

    def calculate_setpoint(self):
        if len(self.setpointVectorX) == len(self.setpointVectorY):
            for i in reversed(range(len(self.setpointVectorX))):
                if self.setpoint_distance >= np.linalg.norm(np.array(self.current_position[0:2]) - np.array([self.setpointVectorX[i], self.setpointVectorY[i]])):
                    setPoint = Point()
                    setPoint.x = self.setpointVectorX[i]/100
                    setPoint.y = self.setpointVectorY[i]/100    
                    self.pub_setPoint.publish(setPoint)
                    # print("setPoint : ")
                    # print(setPoint)
                    break

    def run(self):
            rate = rospy.Rate(20.0)
            while not rospy.is_shutdown():
                self.calculate_setpoint()
                rate.sleep()


if __name__ == "__main__":
    node = SetpointPublisher("SetpointPublisher")
    node.run()
