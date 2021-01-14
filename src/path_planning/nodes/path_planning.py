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

show_animation = False

class PathPlanning():
    def __init__(self,name):
        rospy.init_node(name)

        # Size of Obsticles
        self.box_size = 20  # size of square box
        self.wall_thickness = 10
        self.wall_hole_width = 70

        self.currentPosition = [50, 50]

        self.setpoint_distance = 10

        self.boxCoordinate = np.zeros(2)
        self.bool_box_Detected = False
        self.wallCoordinate = np.zeros(2)
        self.bool_wall_Detected = True

        # Map variables
        self.ox, self.oy = [], []
        self.grid_size = 20  # [cm]
        self.robot_radius = 10  # [cm]

        self.planning_rate = 10

        # Get a destination parameters
        rospy.set_param('destinationX', 75)
        rospy.set_param('destinationY', 250)

        # PUBLISHER:
        self.pub_setPoint= rospy.Publisher("pathPlanning/setpoints2goal", Point, queue_size=1)

        # SUBSCRIBER:
        rospy.Subscriber("/ground_truth/state", Odometry, self.ground_truth_callback, queue_size=1)
        # rospy.Subscriber("objectDetection/coordinate",Point,self.boxDetected)
        rospy.Subscriber("box/pose", Pose, self.box_callback)
        rospy.Subscriber("wall/pose", Pose, self.wall_callback)


    def ground_truth_callback(self, msg):
        self.currentPosition = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])*100

    def box_callback(self, msg):
        tagCoordinate = np.array([msg.position.x, msg.position.y, msg.position.z])*100 # cm
        self.boxCoordinate = (tagCoordinate + self.qv_mult(msg.orientation, [0, -1/2*self.box_size, 0]))[0:2] # middle point
        self.bool_box_Detected = True

    def addBoxToMap(self):
        radius = m.sqrt(2*self.box_size**2)
        if self.bool_box_Detected:
            for i in range(0,150):
                for j in range(0, 340):
                    if (i-self.boxCoordinate[0])**2 + (j-self.boxCoordinate[1])**2 <= radius**2:
                        self.ox.append(i)
                        self.oy.append(j)

    # rotate vector v1 by quaternion q1 
    def qv_mult(self, q1_in, v1_in):
        v1 = v1_in # [v1_in.x, v1_in.y, v1_in.z]
        q1 = [q1_in.x, q1_in.y, q1_in.z, -q1_in.w] # Invert Quaternion
        v1 = tf.transformations.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2), 
            tf.transformations.quaternion_conjugate(q1)
        )[:3]


    def wall_callback(self, msg):
        self.wallCoordinate = np.array([msg.position.x, msg.position.y])*100 # cm
        self.bool_wall_Detected = True

    def addWallToMap(self):
        if self.bool_wall_Detected:
            for i in range(0, 150):
                for j in range(int(round(self.wallCoordinate[1], 0)), int(round(self.wallCoordinate[1] + self.wall_thickness, 0))):
                    if abs(i - self.wallCoordinate[0] - 0.5*self.wall_hole_width) >= self.wall_hole_width*0.5: 
                        self.ox.append(i)
                        self.oy.append(j)

    def staticMap(self):
        self.ox, self.oy = [], []
        # Bottom Wall
        for i in range(0, 150):
            self.ox.append(i)
            self.oy.append(0.0)
        # Left Wall
        for i in range(0, 340):
            self.ox.append(0.0)
            self.oy.append(i)
        # Top Wall
        for i in range(0, 150):
            self.ox.append(i)
            self.oy.append(340)
        # Right Wall
        for i in range(0, 340):
            self.ox.append(150)
            self.oy.append(i)

    def Planning(self):
        # start and goal position
        sx = self.currentPosition[0]  # [cm]
        sy = self.currentPosition[1]  # [cm]
        gx = rospy.get_param('destinationX')  # [cm]
        gy = rospy.get_param('destinationY')  # [cm]
        t = time.time()
        a_star = AStarPlanner(self.ox, self.oy, self.grid_size, self.robot_radius)
        rx, ry = a_star.planning(sx, sy, gx, gy)
        elapsed = time.time() - t
        # print("elapsed time for Astar:  " + str(round(elapsed, 4)))
        path_2d = np.zeros((len(rx), 2))
        for i in range(len(rx)):
            path_2d[i][0] = rx[i]
            path_2d[i][1] = ry[i]
        # path_2d = path_2d[::-1]
        xvals, yvals = bezier_curve(path_2d, nTimes=50)
        self.setpointVectorX = xvals
        self.setpointVectorY = yvals
        setPoint = Point()
        setPoint = self.publishSetpoint()

        if show_animation:  # pragma: no cover
            plt.clf()
            plt.plot(self.ox, self.oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(rx, ry, "-r")
            plt.plot(xvals, yvals, 'o')
            plt.plot(setPoint.x, setPoint.y, "xy")
            #plt.plot(xvals, yvals, 'o', t, p(t), '-')
            plt.pause(0.001)
            plt.show(block=True)
            plt.pause(0.1)


    def publishSetpoint(self):
        if len(self.setpointVectorX) == len(self.setpointVectorY):
            setPoint = Point()
            for i in range(len(self.setpointVectorX)):
                setPoint.x = self.setpointVectorX[i]/100
                setPoint.y = self.setpointVectorY[i]/100
                if self.setpoint_distance <= np.linalg.norm(np.array(self.currentPosition[0:2]) - np.array([self.setpointVectorX[i], self.setpointVectorY[i]])):
                    break
            # print("setPoint : ")
            # print(setPoint)
            self.pub_setPoint.publish(setPoint)
            return setPoint


    def run(self):
        rate = rospy.Rate(self.planning_rate)
        while not rospy.is_shutdown():
            t = time.time()
            self.staticMap()
            self.addBoxToMap()
            self.addWallToMap()
            self.Planning()
            elapsed = time.time() - t
            # print("elapsed time with planning:  " + str(round(elapsed, 4)))
            rate.sleep()
    

def main():
    node = PathPlanning("PathPlanning")
    node.run()

if __name__ == "__main__":
   main()
