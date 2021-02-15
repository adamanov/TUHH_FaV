#!/usr/bin/env python
import rospy
import numpy as np
import time
import math as m

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_processor.msg import Orientation
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class ControllerNode:
    def __init__(self, name):
        rospy.init_node(name)

        if rospy.get_param("sim"):
            self.arm_vehicle()

        self.path_following_active = False
        self.in_depth_range = False

        self.hole_height = 0.5


        self.yaw_current = 0.0

        self.setpoint_distance = 0.1
        
        self.AngleYawDifference = 0.0
        self.currentPosition = [0.0, 0.0, 0.0]

        self.tag_size = 0.1

        # for depth stabilization
        self.min_in_depth_time = 3.0
        self.in_depth_range_since = rospy.get_time()
        self.depth_threshold = 0.3
        self.previous_in_depth_range = False

        # yaw rotation
        self.yaw_adjust_interval = 0.01 # [s]
        self.last_yaw_adjust = 0

        # Destination Points
        self.destinationX = [0.75, 1.0, 0.85, 0.8, 1.0, 1.3]
        self.destinationY = [2.50, 0.80, 2.70, 0.8, 2.8, 0.9]
        self.destinationPointNr = 0

        # Desired position of robot
        self.setpointPoseX = 0.9
        self.setpointPoseY = 0.7
        self.setpointPoseZ = -0.3
        # Set a desired orientation of robot
        self.setpointAngleRoll = 0
        self.setpointAnglePitch = 0
        self.setpointAngleYaw = 0

        # PUBLISHER:
        self.thrust_setpoint_pub = rospy.Publisher("thrust/setpoint", Float64, queue_size=1)
        self.x_setpoint_pub = rospy.Publisher("setpoint/x_inertial", Float64, queue_size=1)
        self.y_setpoint_pub = rospy.Publisher("setpoint/y_inertial", Float64, queue_size=1)
        self.vt_setpoint_pub = rospy.Publisher("vertical_thrust/setpoint", Float64, queue_size=1)
        self.vt_state_pub = rospy.Publisher("vertical_thrust/state", Float64, queue_size=1)
        self.lt_setpoint_pub = rospy.Publisher("lateral_thrust/setpoint", Float64, queue_size=1)
        self.yaw_setpoint_pub = rospy.Publisher("yaw/setpoint", Float64, queue_size=1)
        self.destination_pub = rospy.Publisher("destination_point", Point, queue_size=1)

        # SUBSCRIBER:
        rospy.Subscriber("pathPlanning/setpoints2goal", Point, self.setpoint_callback_xy, queue_size=1)
        rospy.Subscriber("wall/pose", Pose, self.wall_callback)
        rospy.Subscriber("orientation/euler", Orientation, self.euler_callback, queue_size=1)
        self.simulation = rospy.get_param("sim")
        if self.simulation:
            rospy.Subscriber("/ground_truth/state", Odometry, self.ground_truth_callback, queue_size=1)
        else:
            self.EKF_topic_name = rospy.get_param("EKF_topic_name")
            rospy.Subscriber(self.EKF_topic_name, PoseWithCovarianceStamped, self.pose_callback)

    def pose_callback(self, msg):
        self.orientation_rov = msg.pose.pose.orientation
        self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.check_if_at_destination_point()
    
    def arm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        while not arm(True).success:
            rospy.logwarn_throttle(1, "Could not arm vehicle. Keep trying.")
        rospy.loginfo("Armed successfully.")  

    def ground_truth_callback(self, msg):
        self.currentPosition = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.check_if_at_destination_point()

    def euler_callback(self, msg):
        self.yaw_current = msg.yaw

    def setpoint_callback_xy(self, msg):
        self.setpointPoseX = msg.x
        self.setpointPoseY = msg.y

    def wall_callback(self, msg):
        self.setpointPoseZ = msg.position.z - 0.5*self.hole_height - 0.5*self.tag_size
        if self.path_following_active == False:
            print("Turned ON path following (detected wall)")
            self.start_position = self.currentPosition[0:2]
        self.path_following_active = True

    def check_depth(self):      
        if abs(self.setpointPoseZ - self.currentPosition[2]) <= self.depth_threshold:
            if self.previous_in_depth_range == False:
                self.in_depth_range_since = rospy.get_time()
                self.previous_in_depth_range = True
            if (rospy.get_time() - self.in_depth_range_since) >= self.min_in_depth_time:
                if self.in_depth_range == False:
                    print("xy-Controller active (in depth range)")
                self.in_depth_range = True
        else:
            if self.in_depth_range == True:
                print("xy-Controller turned off (out off depth range)")
            self.in_depth_range = False
            self.previous_in_depth_range = False

    def check_if_at_destination_point(self):
        if self.setpoint_distance >= np.linalg.norm(np.array(self.currentPosition[0:2]) - \
        np.array([self.destinationX[self.destinationPointNr], self.destinationY[self.destinationPointNr]])) \
        and self.path_following_active == True:
            self.path_following_active = False
            self.destinationPointNr = (self.destinationPointNr + 1) % len(self.destinationX)
            print("destinationPointNr: " + str(self.destinationPointNr))
            pub_msg = Point()
            pub_msg.x, pub_msg.y = self.destinationX[self.destinationPointNr], self.destinationY[self.destinationPointNr]
            print("New destination point at: " + str(self.destinationX[self.destinationPointNr]) + " " + str(self.destinationY[self.destinationPointNr]))
            self.destination_pub.publish(pub_msg)
            print("Turned OFF path following (at destination point)")

    def tf_calculate_and_publish_yaw(self):
        if self.path_following_active:
            vector_to_destination = np.array([self.destinationX[self.destinationPointNr], self.destinationY[self.destinationPointNr]]) - np.array(self.start_position)
            self.setpointAngleYaw = m.atan(vector_to_destination[1]/vector_to_destination[0])
            if vector_to_destination[0] < 0:
                if vector_to_destination[1] < 0:
                    self.setpointAngleYaw -= m.pi
                else:
                    self.setpointAngleYaw += m.pi
            self.setpointAngleYaw += m.pi
            # print("self.setpointAngleYaw (path_following_active == True): " + str(self.setpointAngleYaw))
        else:
            if (rospy.get_time() - self.last_yaw_adjust > self.yaw_adjust_interval):
                self.setpointAngleYaw = (self.setpointAngleYaw + 1*m.pi*2/360) % (2*m.pi) # between zero an 2*pi
            # print("self.setpointAngleYaw (path_following_active == False): " + str(self.setpointAngleYaw))
        difference = (self.yaw_current + m.pi) - self.setpointAngleYaw
        if m.pi >= abs(difference):
            self.AngleYawDifference = difference
        elif difference >= 0:
            self.AngleYawDifference = difference - 2*m.pi
        elif difference < 0:
            self.AngleYawDifference = difference + 2*m.pi
        # print("self.AngleYawDifference (before min max)" + str(self.AngleYawDifference))
        self.AngleYawDifference = max(min(self.AngleYawDifference, 20*2*m.pi/360), -20*2*m.pi/360)
        # print("self.AngleYawDifference (after min max)" + str(self.AngleYawDifference))
        self.last_yaw_adjust = rospy.get_time()
        self.publish_Float64(self.yaw_setpoint_pub, -self.AngleYawDifference)

    def publish_Float64(self, pub, float):
        msg = Float64()
        msg.data = float
        pub.publish(msg)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.check_depth()
            if self.path_following_active and self.in_depth_range and rospy.get_param("PID-xy-ON"):
                # Publish states and setpoints x,y to PID Controller
                self.publish_Float64(self.x_setpoint_pub, self.setpointPoseX)
                self.publish_Float64(self.y_setpoint_pub, self.setpointPoseY)
                self.publish_Float64(self.lt_setpoint_pub, 0.0 )
                self.publish_Float64(self.thrust_setpoint_pub, 0.0)
            # publish setpoint Z
            self.publish_Float64(self.vt_setpoint_pub, self.setpointPoseZ)
            self.publish_Float64(self.vt_state_pub, self.currentPosition[2])
            # publish desired Yaw ANgle
            self.tf_calculate_and_publish_yaw()
            rate.sleep()


if __name__ == "__main__":
    node = ControllerNode("ControllerNode")
    node.run()
