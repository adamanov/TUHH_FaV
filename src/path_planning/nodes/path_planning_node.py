#!/usr/bin/env python
import rospy
import numpy as np
import math as m
import tf.transformations
import matplotlib.pyplot as plt
import time
from PythonRobotics_Astar import *

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovariance
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from path_planning.msg import PathCoordinates
from geometry_msgs.msg import PoseStamped

show_animation = False

class PathPlanning():
    def __init__(self,name):
        rospy.init_node(name)

        self.planning_rate = 10

        # Size of Obsticle and wall
        self.box_size = 20  # size of square box
        self.wall_thickness = 10
        self.wall_hole_width = 80
        self.april_tag_width = 10

        # size of tank
        self.tank_width = 170 
        self.tank_length = 345

        # Initialize
        self.currentPosition = [50, 50]
        self.boxCoordinate = np.zeros(2)
        self.bool_box_Detected = False
        self.wallCoordinate = np.zeros(2)
        self.bool_wall_Detected = False

        # Map variables
        self.gx, self.gy = 75, 250
        self.ox, self.oy = [], []
        self.grid_size = 5  # [cm]
        self.robot_radius = 30  # [cm]

        # PUBLISHER:
        self.pub_path = rospy.Publisher("path/coordinates", PathCoordinates, queue_size=1)

        # SUBSCRIBER:
        self.simulation = rospy.get_param("sim")
        if self.simulation:
            rospy.Subscriber("/ground_truth/state", Odometry, self.position_callback, queue_size=1)
        else:
            self.EKF_topic_name = rospy.get_param("EKF_topic_name")
            rospy.Subscriber(self.EKF_topic_name, PoseWithCovarianceStamped, self.position_callback)
        rospy.Subscriber("box/pose", Pose, self.box_callback)
        rospy.Subscriber("wall/pose", Pose, self.wall_callback)
        rospy.Subscriber("destination_point", Point, self.destination_point_callback, queue_size=1)

    def destination_point_callback(self, msg):
        self.gx, self.gy = msg.x*100, msg.y*100
        # print("gx: " + str(self.gx) + "    gy: " + str(self.gy))

    def position_callback(self, msg):
        self.currentPosition = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])*100

    def box_callback(self, msg):
        tagCoordinate = np.array([msg.position.x, msg.position.y, msg.position.z])*100 # cm
        self.boxCoordinate = (tagCoordinate + self.qv_mult(msg.orientation, [0, -1/2*self.box_size, 0]))[0:2] # middle point
        self.bool_box_Detected = True

    def addBoxToMap(self):
        if self.bool_box_Detected:
            radius = m.sqrt(2*(self.box_size*0.5)**2)
            for i in range(0,self.tank_width):
                for j in range(0, self.tank_length):
                    if (i-self.boxCoordinate[0])**2 + (j-self.boxCoordinate[1])**2 <= radius**2:
                        self.ox.append(i)
                        self.oy.append(j)

    def wall_callback(self, msg):
        self.wallCoordinate = np.array([msg.position.x, msg.position.y])*100 # cm
        self.bool_wall_Detected = True

    def addWallToMap(self):
        if self.bool_wall_Detected:
            for i in range(0, self.tank_width):
                for j in range(int(round(self.wallCoordinate[1], 0)), int(round(self.wallCoordinate[1] + self.wall_thickness, 0))):
                    center_of_hole = self.wallCoordinate[0] + 0.5*self.wall_hole_width + 0.5*self.april_tag_width
                    if abs(i - center_of_hole) >= self.wall_hole_width*0.5: 
                        self.ox.append(i)
                        self.oy.append(j)

    def staticMap(self):
        self.ox, self.oy = [], []
        # Bottom Wall
        for i in range(0, self.tank_width):
            self.ox.append(i)
            self.oy.append(0.0)
        # Left Wall
        for i in range(0, self.tank_length):
            self.ox.append(0.0)
            self.oy.append(i)
        # Top Wall
        for i in range(0, self.tank_width):
            self.ox.append(i)
            self.oy.append(self.tank_length)
        # Right Wall
        for i in range(0, self.tank_length):
            self.ox.append(self.tank_width)
            self.oy.append(i)

    def Planning(self):
        # start and goal position
        sx = self.currentPosition[0]  # [cm]
        sy = self.currentPosition[1]  # [cm]
        # t = time.time()
        a_star = AStarPlanner(self.ox, self.oy, self.grid_size, self.robot_radius)
        rx, ry = a_star.planning(sx, sy, self.gx, self.gy)
        # elapsed = time.time() - t
        # print("elapsed time for Astar:  " + str(round(elapsed, 4)))
        path_2d = np.zeros((len(rx), 2))
        for i in range(len(rx)):
            path_2d[i][0] = rx[i]
            path_2d[i][1] = ry[i]
        # path_2d = path_2d[::-1]
        xvals, yvals = bezier_curve(path_2d, nTimes=50)
        self.setpointVectorX = xvals
        self.setpointVectorY = yvals
        self.publishSetpoint()

        if show_animation:  # pragma: no cover
            plt.clf()
            plt.plot(self.ox, self.oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(self.gx, self.gy, "xb")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(rx, ry, "-r")
            plt.plot(xvals, yvals, 'o')
            # plt.text(0.1, 0.9, str(round(self.currentPosition[2], 1)),
            #     horizontalalignment='center',
            #     verticalalignment='center', color='purple', size=10)  
            #plt.plot(xvals, yvals, 'o', t, p(t), '-')
            plt.pause(0.001)
            plt.show(block=False)
            plt.pause(1)
            plt.close()
            

    def publishSetpoint(self):
        msg = PathCoordinates()
        msg.x_array = self.setpointVectorX
        msg.y_array = self.setpointVectorY
        self.pub_path.publish(msg)

    # rotate vector v1 by quaternion q1 
    def qv_mult(self, q1_in, v1_in):
        v1 = v1_in # [v1_in.x, v1_in.y, v1_in.z]
        q1 = [q1_in.x, q1_in.y, q1_in.z, -q1_in.w] # Invert Quaternion
        v1 = tf.transformations.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2), 
            tf.transformations.quaternion_conjugate(q1))[:3]

    def run(self):
        rate = rospy.Rate(self.planning_rate)
        while not rospy.is_shutdown():
            self.staticMap()
            self.addBoxToMap()
            self.addWallToMap()
            if self.bool_wall_Detected:
                self.Planning()
            rate.sleep()
    

def main():
    node = PathPlanning("PathPlanning")
    node.run()

if __name__ == "__main__":
   main()
