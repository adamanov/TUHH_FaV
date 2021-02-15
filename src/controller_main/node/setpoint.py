#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs import Point
from geometry_msgs import pose



class SetpointNode():
    def __init__(self):

        rospy.init_node("SetpointNode")
        self.start_time = rospy.get_time()


        self.setpoint_pose_pub = rospy.Publisher("desired_pose/setpoint",
                                            Point,
                                            queue_size=1)

        self.setpoint_angle_pub = rospy.Publisher("desired_angle/setpoint",
                                            Point,
                                            queue_size=1)         

        rospy.set_param("x",0.5)
        rospy.set_param("y",0.5)
        rospy.set_param("z",0.5)
        rospy.set_param("yaw",0)
        rospy.set_param("roll",0)
        rospy.set_param("pitch",0)                                                                       

   

    def publish_setpoint(self):
 
        Pose = Point()
        angle = Point()
        Pose.x = rospy.get_param("x")
        Pose.y = rospy.get_param("y")
        Pose.z = rospy.get_param("z")
        angle.x = rospy.get_param("roll")
        angle.y = rospy.get_param("pitch")
        angle.z = rospy.get_param("yaw")


        # msg = Float64()
        # msg.data = setpoint
        setpoint_pose_pub.publish(pose)
        setpoint_angle_pub.publish(angle)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.get_setpoint()
            rate.sleep()


def main():
    node = SetpointNode()
    node.run()


if __name__ == "__main__":
    main()
