#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np


class DepthSetpointNode():
    def __init__(self):
        rospy.init_node("depth_setpoint")

        # self.upper_bound_sp = -0.2
        # self.lower_bound_sp = -0.7
        self.setpoint = -0.7

        rospy.set_param('safezone_upper', -0.15)
        rospy.set_param('safezone_lower', -0.6)

        rospy.set_param('setpoint', -0.5)  # for setting setpoint use -> rosparam set /setpoint -1.0

        self.switch_time = 10
        self.last_switch = 0

        self.setpoint_pub = rospy.Publisher("depth/setpoint",
                                            Float64,
                                            queue_size=1)
        #rospy.Subscriber("depth_setpoint", Float64, self.setpoint_callback, queue_size=1)
    
    def calculate_setpoint(self):
        if self.isValidSetpoint():
            self.setpoint = rospy.get_param("setpoint")
        else:
            rospy.loginfo("Please set Setpoint to float in safezone!")
        # if (self.switch_time < (rospy.Time.now().secs - self.last_switch)):
        #     self.last_switch = rospy.Time.now().secs
        #     if (self.setpoint == self.upper_bound_sp):
        #         self.setpoint = self.lower_bound_sp
        #     else:
        #         self.setpoint = self.upper_bound_sp

    
    def isValidSetpoint(self):
        return (rospy.get_param("setpoint") < rospy.get_param("safezone_upper")) and (rospy.get_param("setpoint") > rospy.get_param("safezone_lower"))


    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            msg = Float64()
            self.calculate_setpoint()
            # fill msg with example setpoint _ Wie hier uebermitteln
            msg.data = self.setpoint
            self.setpoint_pub.publish(msg)
            rate.sleep()


def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
