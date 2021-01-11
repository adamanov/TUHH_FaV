#!/usr/bin/env python
import rospy
import numpy as np
from PythonRobotics_Astar import *


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovariance
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement
from std_msgs.msg import Float64


class PathPlanning():
    def __init__(self,name):
        rospy.init_node(name)

        # Map variables
        self.ox, self.oy = [], []
        self.staticMap()

        rospy.set_param('prediction_rate',20.0)
        self.prediction_rate = rospy.get_param('prediction_rate')

        # Get a destination parameters
        self.destination = []
        rospy.set_param('destinationX',5)
        rospy.set_param('destinationY',10)
        rospy.set_param('destinationZ',1)
        self.destination[0] = rospy.get_param('destinationX')
        self.destination[1] = rospy.get_param('destinationY')
        self.destination[2] = rospy.get_param('destinationZ')

        # Calcualted Setpoints From PathPlanning
        setpointVectorX = []
        setpointVectorY = []
        setpointVectorZ = []

        self.box = np.array[[10,10],[10,10]]
        rospy.Subscriber("localization/ekf/baselink_position", Point, self.positionCallback)
        self.pub_setPoint= rospy.Publisher("pathPlanning/setpoints2goal", Point, queue_size=1)

        rospy.Subscriber("objectDetection/coordinate",Point,self.boxDetected)

    def positionCallback(self,msg):
        currentPosition = [msg.x,msg.y,msg.z]


    def boxDetected(self,msg):
        boxCoordinate = [msg.x, msg.y, msg.z]
        self.updateMap(boxCoordinate)
        self.Planning()
        self.publishSetpoint()

    def updateMap(self, boxCoordinate):
        box = 20  # square box
        boxCoordinate = np.array([boxCoordinate[0], boxCoordinate[1]])
        print(boxCoordinate[1])
        for i in range(0,150):
            if abs(i- boxCoordinate[0]) < box/2.0:
                self.ox.append(i)
                self.oy.append(boxCoordinate[1])
            if abs(i - boxCoordinate[0]) < box / 2.0:
                self.ox.append(i)
                self.oy.append(box+boxCoordinate[1])

        for i in range(0,340):
            if abs(i- boxCoordinate[1]) < box/2.0:
                self.ox.append(box/2 + boxCoordinate[0])
                self.oy.append(i+box/2)
            if abs(i- boxCoordinate[1]) < box/2.0:
                self.ox.append(-box/2 + boxCoordinate[0])
                self.oy.append(i+box/2)


    def staticMap(self):
        # set obstacle positions

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

        for i in range(0, 60):
            self.ox.append(i)
            self.oy.append(200)
            self.ox.append(i)
            self.oy.append(50)
        for i in range(0, 60):
            # ox.append(150 - i)
            # oy.append(200)
            self.ox.append(150 - i)
            self.oy.append(225)


    def Planning(self):
        grid_size = 25  # [m]
        robot_radius = 10  # [m]

        # start and goal position
        sx = self.currentPosition[0]  # [cm]
        sy = self.currentPosition[1]  # [cm]
        gx = self.destination[0]  # [cm]
        gy = self.destination[1]  # [cm]

        a_star = AStarPlanner(self.ox, self.oy, grid_size, robot_radius)
        rx, ry = a_star.planning(sx, sy, gx, gy)

        path_2d = np.zeros((len(rx), 2))
        for i in range(len(rx)):
            path_2d[i][0] = rx[i]
            path_2d[i][1] = ry[i]
        path_2d = path_2d[::-1]
        print(path_2d)

        xvals, yvals = bezier_curve(path_2d, nTimes=50)
        print("rx", rx)
        print("ry", ry)
        self.setpointVectorX = xvals
        self.setpointVectorY = yvals



    def publishSetpoint(self):
        if len(self.setpointVectorX) == len(self.setpointVectorY):
            for i in range(len(self.setpointVectorX)):
                setPoint = Point()
                setPoint.x = self.setpointVectorX[i]
                setPoint.y = self.setpointVectorY[i]
                self.pub_setPoint.publish(setPoint)

    def run(self):
        rate = rospy.Rate(self.prediction_rate)
        while not rospy.is_shutdown():
            self.publishSetpoint()
            rate.sleep()

def main():
    node = PathPlanning("PathPlanning")
    node.run()

if __name__ == "__main__":
   main()
