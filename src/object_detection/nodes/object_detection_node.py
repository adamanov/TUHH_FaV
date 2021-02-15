#!/usr/bin/env python
import rospy
import numpy as np
import tf.transformations as tf
from rotations import *

from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_processor.msg import Orientation


class ObjectDetectionNode():
    def __init__(self, name):
        rospy.init_node(name)

        self.current_position = np.zeros(2)
        self.orientation_rov = Orientation()

        self.baselink_to_camera = np.array([0.2, 0, 0.1])

        # for path planning testing
        self.test_path_planning = rospy.get_param("test_path_planning")
        self.pub_rate = 20
        self.test_wall_position = np.array([0.35, 1.6, -0.4])
        self.measured_wall_position = Point()
        self.measured_wall_position.x, self.measured_wall_position.y, self.measured_wall_position.z,= 0.35, 1.64, -0.35

        # PUBLISHER:
        self.box_pub = rospy.Publisher("box/pose", Pose, queue_size=1)
        self.wall_pub = rospy.Publisher("wall/pose", Pose, queue_size=1)
        self.pub_meas_wall = rospy.Publisher("wall/pose/measured", Point, queue_size=1)
        
        #SUBSCRIBER:
        self.simulation = rospy.get_param("sim")
        if self.simulation:
            rospy.Subscriber("tag_object_detections_sim", AprilTagDetectionArray, self.on_apriltag, queue_size=1)
            rospy.Subscriber("/ground_truth/state", Odometry, self.pose_callback, queue_size=1)
        else:
            rospy.Subscriber("/klopsi/front_camera/tag_detections", AprilTagDetectionArray, self.on_apriltag, queue_size=1)
            self.EKF_topic_name = rospy.get_param("EKF_topic_name")
            rospy.Subscriber(self.EKF_topic_name, PoseWithCovarianceStamped, self.pose_callback)

    def pose_callback(self, msg):
        self.orientation_rov = msg.pose.pose.orientation
        self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def on_apriltag(self, tag_array_msg):
        if not self.test_path_planning:
            num_tags = len(tag_array_msg.detections)
            if num_tags:
                msg = Pose()
                for tag in tag_array_msg.detections:
                    base_to_sensor_inertial = rotate_vector_by_quaternion(self.baselink_to_camera, \
                        convert_quaternion_to_array(self.orientation_rov))
                    sensor_to_tag_inertial = rotate_vector_by_quaternion(convert_point_to_array(tag.pose.pose.pose.position), \
                        convert_quaternion_to_array(self.orientation_rov))
                    msg.position = convert_array_to_point(self.current_position + base_to_sensor_inertial + sensor_to_tag_inertial)
                    msg.orientation = convert_array_to_quaternion(convert_quaternion_to_globalsytem(
                        convert_quaternion_to_array(self.orientation_rov), \
                        convert_quaternion_to_array(tag.pose.pose.pose.orientation)))
                    if tag.id[0] == 0 or tag.id[0] == 2:
                        self.wall_pub.publish(msg)
                        # print(tag.id[0])
                        # print(tag.pose.pose.pose.position)
                        # print(tf.euler_from_quaternion(convert_quaternion_to_array(tag.pose.pose.pose.orientation)))
                        # print(" ")
                    elif 3 <= tag.id[0] <= 10:
                        self.box_pub.publish(msg)

    def pub_virtual_wall_coordinates(self):
        msg = Pose()
        msg.position = convert_array_to_point(self.test_wall_position)
        msg.orientation = convert_array_to_quaternion(tf.quaternion_from_euler(1.57, -0.0, 0.0))
        self.wall_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            if self.test_path_planning:
                self.pub_virtual_wall_coordinates()
            else:
                self.pub_meas_wall.publish(self.measured_wall_position)
            rate.sleep()


def main():
    node = ObjectDetectionNode("ObjectDetection")
    node.run()


if __name__ == "__main__":
    main()
