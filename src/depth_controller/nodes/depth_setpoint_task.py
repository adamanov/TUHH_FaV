#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64


class DepthSetpointNode():
    def __init__(self):
        rospy.init_node("depth_setpoints")
        self.start_time = rospy.get_time()
        self.sinusoidal = rospy.get_param('~sinusoidal', False)

        self.setpoint_1 = rospy.get_param('~setpoint_1', -0.3)
        self.setpoint_2 = rospy.get_param('~setpoint_2', -0.6)
        self.duration = rospy.get_param('~duration', 30.0)

        self.sin_amplitude = rospy.get_param('~amplitude', 0.2)
        self.sin_omega = (2 * np.pi) / (rospy.get_param('~period', 10.0))
        self.sin_offset = rospy.get_param('~offset', -0.45)

        self.setpoint_pub = rospy.Publisher("depth_setpoint",
                                            Float64,
                                            queue_size=1)

    def get_setpoint(self):
        now = rospy.get_time()
        time = self.start_time - now

        if self.sinusoidal:
            setpoint = self.sin_amplitude * np.sin(
                self.sin_omega * time) + self.sin_offset
        else:
            # jump between two setpoints
            i = time % (self.duration * 2)
            if i > (self.duration):
                setpoint = self.setpoint_1
            else:
                setpoint = self.setpoint_2

        self.publish_setpoint(setpoint)

    def publish_setpoint(self, setpoint):
        rospy.set_param("setpoint", setpoint)
        # msg = Float64()
        # msg.data = setpoint
        # self.setpoint_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.get_setpoint()
            rate.sleep()


def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
